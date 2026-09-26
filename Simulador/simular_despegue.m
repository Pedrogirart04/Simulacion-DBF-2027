function [x,y,z,v_x,v_y,v_z,pitch_rad,yaw_rad,t,Energy,inform_despegue,n_pasos] = simular_despegue( ...
    MTOW, dt, rho, S_ref, cd0, CL_max, CL_suelo, mu_rodadura, k_rot, gamma_obj_deg, z_objetivo, ...
    throttle_despegue, PROP_TABLE, MOTOR_TABLE, AVION_TABLE, S_Banner, cd_Banner, wn, max_pitch_rate, crud)
% SIMULAR_DESPEGUE Integra rodaje + rotación + climb-out + nivelación.
% Devuelve el estado final (listo como condición inicial de la Sección 5)
% y una matriz de log compatible con "inform" (33 filas).
%
% Simplificación: viento calmo durante todo el despegue.
    if nargin < 20 || isempty(crud)
        crud = 1;
    end

    g = 9.81;
    gamma_obj = deg2rad(gamma_obj_deg);

    % --- Batería y motor (mismos valores que en airplane_dynamics_opt) ---
    ncells = 8; Q = 3.3; E0 = 4.19; K = 0.02; A = 0.2; B = 4; Rbat = 0.024;
    Kt = 0.0308; Ke = 0.0308; Rint = 0.00865; I0 = 1.71;
    omega_seed = 300;

    throttle_real = (throttle_despegue - 1225)/(2000 - 1225);

    V_stall   = sqrt(2*MTOW*g / (rho*S_ref*CL_max));
    V_takeoff = k_rot * V_stall;

    [~, Avion_fila_suelo] = avion_cl(AVION_TABLE, CL_suelo);
    CD_suelo = Avion_fila_suelo.c_d + cd0;

    max_pasos_despegue = 2000;
    inform_despegue = zeros(33, max_pasos_despegue);
    n_pasos = 0;

    x = 0; y = 0; z = 0;
    V = 0;
    pitch_rad = 0;
    pitch_rate = 0;
    yaw_rad = 0;
    Energy = 0;
    t = 0;

    fprintf('--- DESPEGUE ---\n');
    fprintf('V_stall = %.1f m/s | V_takeoff = %.1f m/s | gamma_obj = %.0f° | z_obj = %.0f m\n', ...
            V_stall, V_takeoff, gamma_obj_deg, z_objetivo);

    % =====================================================================
    % FASE 1: RODAJE
    % =====================================================================
    while V < V_takeoff
        v_air = max(V, 1e-3);

        Vocv = battery_cntm_local(Energy, ncells, Q, E0, K, A, B);
        omega_eq = motor_prop_eq_local(throttle_real, Vocv, v_air, PROP_TABLE, Kt, Ke, Rint, I0, Rbat, omega_seed);
        omega_seed = omega_eq;
        Prop_fila = interpProp(PROP_TABLE, v_air, omega_eq*60/(2*pi));
        Thrust_N = Prop_fila.Thrust_N;

        I_motor = max((throttle_real*Vocv - Ke*omega_eq)/(Rint + throttle_real^2*Rbat), 0);
        I_batt  = throttle_real * I_motor;

        Drag     = (0.5*rho*S_ref*CD_suelo*V^2 + 0.5*S_Banner*rho*cd_Banner*V^2) * crud;
        Friccion = mu_rodadura * MTOW * g;
        a_long   = (Thrust_N - Drag - Friccion) / MTOW;

        n_pasos = n_pasos + 1;
        inform_despegue(:, n_pasos) = [x;0;0; V;0;0; a_long;0;0; t; 0;CD_suelo;Drag;Thrust_N; ...
            0;0;0; Thrust_N;0;0; -Drag;0;0; I_batt; 0;0;0; 0; omega_eq; 3; NaN; throttle_despegue; 0];

        V = V + a_long*dt;
        x = x + V*dt;
        Energy = Energy + (I_batt/3600)*dt;
        t = t + dt;
    end

    % =====================================================================
    % FASE 2: ROTACIÓN + CLIMB-OUT + NIVELACIÓN
    % =====================================================================
    nivelando = false;
    while true
        target_pitch = gamma_obj;
        if nivelando
            target_pitch = 0;
        end

        % --- Filtro de 2do orden crítico (idéntico al de roll) ---
        e0 = pitch_rad - target_pitch;
        v0 = pitch_rate;
        exp_term = exp(-wn*dt);
        e_new = (e0 + (v0 + wn*e0)*dt) * exp_term;
        v_new = (v0 - wn*(v0 + wn*e0)*dt) * exp_term;
        pitch_rate = sign(v_new) * min(abs(v_new), max_pitch_rate);
        pitch_rad  = target_pitch + e_new;

        v_air = max(V, 1e-3);

        Vocv = battery_cntm_local(Energy, ncells, Q, E0, K, A, B);
        omega_eq = motor_prop_eq_local(throttle_real, Vocv, v_air, PROP_TABLE, Kt, Ke, Rint, I0, Rbat, omega_seed);
        omega_seed = omega_eq;
        Prop_fila = interpProp(PROP_TABLE, v_air, omega_eq*60/(2*pi));
        Thrust_N = Prop_fila.Thrust_N;

        I_motor = max((throttle_real*Vocv - Ke*omega_eq)/(Rint + throttle_real^2*Rbat), 0);
        I_batt  = throttle_real * I_motor;

        % --- Sustentación de ascenso: L = W*cos(pitch) (¡no divide, multiplica!) ---
        lift_required = MTOW*g*cos(pitch_rad);
        v_safe_sq = max(V^2, 1e-6);
        cl = 2*lift_required / (rho*v_safe_sq*S_ref);
        cd0_local = cd0;
        if cl > CL_max
            warning('DESPEGUE - STALL EN CLIMB: CL_req=%.2f > CL_max=%.2f | V=%.1f m/s | pitch=%.1f°', ...
                    cl, CL_max, V, rad2deg(pitch_rad));
            cl = CL_max;
            cd0_local = cd0 + 0.15;
        end
        [~, Avion_fila] = avion_cl(AVION_TABLE, cl);
        cd_total = Avion_fila.c_d + cd0_local;
        Lift = lift_required;
        Drag = (0.5*rho*S_ref*cd_total*V^2 + 0.5*S_Banner*rho*cd_Banner*V^2) * crud;

        thrust_vec = [Thrust_N*cos(pitch_rad); 0; Thrust_N*sin(pitch_rad)];
        drag_vec   = [-Drag*cos(pitch_rad);    0; -Drag*sin(pitch_rad)];
        lift_vec   = [-Lift*sin(pitch_rad);    0;  Lift*cos(pitch_rad)];
        a_vec = [0;0;-g] + (thrust_vec + lift_vec + drag_vec)/MTOW;

        n_pasos = n_pasos + 1;
        inform_despegue(:, n_pasos) = [x;0;z; V*cos(pitch_rad);0;V*sin(pitch_rad); a_vec(1);a_vec(2);a_vec(3); t; ...
            cl;cd_total;Drag;Thrust_N; lift_vec(1);lift_vec(2);lift_vec(3); ...
            thrust_vec(1);thrust_vec(2);thrust_vec(3); drag_vec(1);drag_vec(2);drag_vec(3); ...
            I_batt; 0;0;0; 0; omega_eq; 3; NaN; throttle_despegue; pitch_rad];

        dV = (Thrust_N - Drag - MTOW*g*sin(pitch_rad)) / MTOW;
        V = V + dV*dt;
        x = x + V*cos(pitch_rad)*dt;
        z = z + V*sin(pitch_rad)*dt;
        Energy = Energy + (I_batt/3600)*dt;
        t = t + dt;

        if ~nivelando && z >= z_objetivo
            nivelando = true;
        end
        if nivelando && abs(pitch_rad) < deg2rad(0.5) && abs(pitch_rate) < deg2rad(0.5)
            break;
        end
    end

    inform_despegue = inform_despegue(:, 1:n_pasos);

    v_x = V*cos(pitch_rad);
    v_y = 0;
    v_z = V*sin(pitch_rad);
    pitch_rad = 0;   % ya nivelado, entrega en crucero
end

function Vocv = battery_cntm_local(Ah_consumidos,ncells,Q,E0,K,A,B)
it = min(max(Ah_consumidos,0), Q*0.999);
Vocv_cell = E0 - K*(Q/(Q-it))*it + A*exp(-B*it);
Vocv = ncells*Vocv_cell;
end

function omega_eq = motor_prop_eq_local(throttle_real, Vocv, v_air, PROP_TABLE,Kt,Ke,Rint,I0,Rbat,omega_seed)
f = @(w) (Kt*(max((throttle_real*Vocv - Ke*w)/(Rint + throttle_real^2*Rbat),0) - I0)) - ...
         abs(getfield(interpProp(PROP_TABLE, v_air, w*60/(2*pi)), 'Torque_Nm'));
opts = optimset('Display','off','TolX',1e-6);
    try
        omega_eq = fzero(f, max(omega_seed, 1), opts);
    catch
        w_grid = linspace(1, 3000, 60);
        f_grid = arrayfun(f, w_grid);
        idx = find(sign(f_grid(1:end-1)) ~= sign(f_grid(2:end)), 1, 'first');
        if isempty(idx)
            omega_eq = 0;
        else
            omega_eq = fzero(f, [w_grid(idx), w_grid(idx+1)], opts);
        end
    end
    omega_eq = max(omega_eq, 0);
end