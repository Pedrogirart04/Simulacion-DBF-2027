function [x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,log_step,Energy,t] = airplane_dynamics_opt(MTOW,t_n,ro,S_ref,x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,throttle,cd0, ...
                                                                                          PROP_TABLE, MOTOR_TABLE,AVION_TABLE, ...
                                                                                          Energy,t,S_Banner,cd_Banner,CL_max)

    % --- CONFIGURACIÓN DE VECTORES DE ESTADO PARA RK4 ---
    if nargin < 23 || isempty(CL_max)
        CL_max = 0.62;  % valor legacy, se usa si no se pasa (compatibilidad con Legacy/CONDOR_M1.m)
    end

    P = [x; y; z];
    V = [v_x; v_y; v_z];
    Y = yaw_rad;
    E = Energy;
    
    % Inicialización viento
    persistent x_gust;
    if isempty(x_gust)
        x_gust = zeros(5,1); 
    end
    noise = randn(3,1)/sqrt(t_n);

    persistent omega_seed;
    if isempty(omega_seed)
        omega_seed = 300; % rad/s, valor de arranque genérico, ajustalo si hace falta
    end

    % --- PASO 1: k1 ---
   [dP1, dV1, dY1, dE1, dx_gust1, extra] = compute_derivatives(P, V, Y, x_gust);

    % --- PASO 2: k2 ---
    P2 = P + 0.5 * t_n * dP1;
    V2 = V + 0.5 * t_n * dV1;
    Y2 = Y + 0.5 * t_n * dY1;
    x_gust2 = x_gust + 0.5 * t_n * dx_gust1;
    [dP2, dV2, dY2, dE2, dx_gust2, ~] = compute_derivatives(P2, V2, Y2, x_gust2);

    % --- PASO 3: k3 ---
    P3 = P + 0.5 * t_n * dP2;
    V3 = V + 0.5 * t_n * dV2;
    Y3 = Y + 0.5 * t_n * dY2;
    x_gust3 = x_gust + 0.5 * t_n * dx_gust2;
    [dP3, dV3, dY3, dE3, dx_gust3, ~] = compute_derivatives(P3, V3, Y3, x_gust3);

    % --- PASO 4: k4 ---
    P4 = P + t_n * dP3;
    V4 = V + t_n * dV3;
    Y4 = Y + t_n * dY3;
    x_gust4 = x_gust + t_n * dx_gust3;
    [dP4, dV4, dY4, dE4, dx_gust4, ~] = compute_derivatives(P4, V4, Y4, x_gust4);

    % --- INTEGRACIÓN FINAL (PROMEDIO PONDERADO RK4) ---
    P = P + (t_n / 6) * (dP1 + 2*dP2 + 2*dP3 + dP4);
    V = V + (t_n / 6) * (dV1 + 2*dV2 + 2*dV3 + dV4);
    Y = Y + (t_n / 6) * (dY1 + 2*dY2 + 2*dY3 + dY4);
    E = E + (t_n / 6) * (dE1 + 2*dE2 + 2*dE3 + dE4);
    x_gust = x_gust + (t_n / 6) * (dx_gust1 + 2*dx_gust2 + 2*dx_gust3 + dx_gust4);

    % Variables de salida
    x = P(1); y = P(2); z = P(3);
    v_x = V(1); v_y = V(2); v_z = V(3);
    yaw_rad = Y;
    Energy = E;
    t = t + t_n;

    % Registro final de variables (+ Costo Computo Innecesario)
    %[~, ~, ~, ~, ~, extra] = compute_derivatives(P, V, Y, C_caos);


    % --- LOGGEO EN LA MATRIZ INFORM ---

    %Matriz Inform: Contenidos
    % (1) x [m]              (2) y [m]              (3) z [m]
    % (4) v_x [m/s]           (5) v_y [m/s]           (6) v_z [m/s]
    % (7) a_x [m/s^2]         (8) a_y [m/s^2]         (9) a_z [m/s^2]
    % (10) t [s]              (11) cl                 (12) cd_total
    % (13) drag [N]           (14) Thrust_N [N]       (15) lift_vec_x [N]
    % (16) lift_vec_y [N]     (17) lift_vec_z [N]     (18) thrust_vec_x [N]
    % (19) thrust_vec_y [N]   (20) thrust_vec_z [N]   (21) drag_vec_x [N]
    % (22) drag_vec_y [N]     (23) drag_vec_z [N]     (24) Corriente_real [A]
    % (25) viento_x [m/s]     (26) viento_y [m/s]     (27) viento_z [m/s]
    % (28) roll_rad [rad]     (29) omega (motor) [rad/s]

   log_step = [x; y; z; v_x; v_y; v_z; extra.a(1); extra.a(2); extra.a(3);
             t; extra.cl; extra.cd_total; extra.drag; extra.Thrust_N; ...
             extra.lift_vec(1); extra.lift_vec(2); extra.lift_vec(3); ...
             extra.thrust_vec(1); extra.thrust_vec(2); extra.thrust_vec(3); ...
             extra.drag_vec(1); extra.drag_vec(2); extra.drag_vec(3); ...
             extra.Corriente_real; ...
             extra.v_w(1); extra.v_w(2); extra.v_w(3); ...
             roll_rad; extra.omega];


%% =====================================================================
    %% FUNCIÓN ANIDADA: CÁLCULO DE DERIVADAS TRIDIMENSIONALES + CONFIG VIENTO
    %% =====================================================================
    function [dP, dV, dY, dE, dx_gust, extra] = compute_derivatives(P_curr, V_curr, Y_curr, x_gust_curr)
        
        %Coordenadas
        cos_p=cos(pitch_rad); sin_p=sin(pitch_rad);
        T_pitch=[cos_p 0 -sin_p; 0 1 0; sin_p 0 cos_p];
        cos_r=cos(roll_rad); sin_r=sin(roll_rad);
        T_roll=[1 0 0; 0 cos_r -sin_r; 0 sin_r cos_r];
        cos_y=cos(Y_curr); sin_y=sin(Y_curr);
        T_yaw=[cos_y -sin_y 0; sin_y cos_y 0; 0 0 1];
        
        T_total = T_yaw * T_pitch * T_roll;
        
        % viento
        %parametros
        Va_0 = 25;
        wind_steady = [5;-2;0];
        turbulance = 'light';

        %calculo v_w
        [v_w_i,dx_gust,~] = dryden_wind(x_gust_curr,Va_0,T_total,wind_steady,turbulance,noise);

        % --- VELOCIDADES RELATIVAS (El viento se resta en el plano horizontal) ---
        
        V_a = [V_curr(1); V_curr(2); V_curr(3)] - v_w_i; %v_airspeed
        v_a_mag = norm(V_a);
        v_safe = max(v_a_mag, 1e-6);
        
        versor_a = V_a / v_safe; %esta es la direccion del airspeed
        v_normal_helice = abs(dot(T_total * [1;0;0], V_a));
        
        
        % Modelo de Motor y Hélice
        throttle_real = (throttle - 1225)/(2000-1225); % Cambio el throttle para que el valor sea de 0 a 1
       % Datos de la bateria 
        ncells = 8;
        Q = 3.3;
        E0 = 4.19;
        K = 0.02;
        A = 0.2;
        B = 4;
        Rbat = 0.024;
        % omega_seed =; Si se quiere poner otra velocidad angular incial
        % que no sea 300rad/s

        % Datos del motor (Scorpion A-5025-310kv, fiteo Kt/Ke/Rint/I0)
        Kt   = 0.0308;   % [Nm/A] constante de torque
        Ke   = 0.0308;   % [V*s/rad] constante de fcem (=Kt en SI)
        Rint = 0.00865;  % [ohm] resistencia interna del bobinado
        I0   = 1.71;     % [A] corriente sin carga
        Vocv = battery_cntm(E,ncells,Q,E0,K,A,B); % Tension de circuito abierto o fuente ideal dependiente del estado de carga
        
        omega_eq = motor_prop_eq(throttle_real,Vocv,v_normal_helice,PROP_TABLE,Kt,Ke,Rint,...
                                 I0,Rbat,omega_seed); %Resuelvo el equilibrio de torque
        omega_seed = omega_eq; % Actualizo el valor de las RPM

        RPM_actual = omega_eq*60/(2*pi);
        Prop_fila  = interpProp(PROP_TABLE, v_normal_helice, RPM_actual);
        Thrust_N   = Prop_fila.Thrust_N;
        
         % Corriente de motor (la que circula por Rint) y de batería
        I_motor = (throttle_real*Vocv - Ke*omega_eq) / ...
                  (Rint + throttle_real^2*Rbat);
        I_motor = max(I_motor, 0); % la hélice no puede "motorizar" el motor acá
        I_batt  = throttle_real * I_motor;

        % Corriente_real se mantiene como corriente de BATERÍA (para SOC/Ah),
        % que es la que corresponde integrar en dE.
        Corriente_real = I_batt;
       
        
        % Aerodinámica
        v_safe_sq = max(v_safe^2, 1e-6);
        cos_p_safe = max(cos_p, 1e-6);
        cos_r_safe = max(cos_r, 1e-6);
        g = 9.81;
        lift_required = (MTOW * g) / (cos_r_safe * cos_p_safe);

        cl = (2 * lift_required) / (ro * v_safe_sq * S_ref);
        
        % --- STALL CHECK ---
        if cl > CL_max
            warning('STALL: CL_req = %.2f > CL_max = %.2f | V = %.1f m/s | roll = %.1f°', ...
                    cl, CL_max, v_safe, rad2deg(roll_rad));
            cl = CL_max;   % Recortar a CLmax — el avión no puede generar más
            cd0 = cd0 + 0.15; %disparamos cd por separacion violenta de flujo
        end
        [~, Avion_fila] = avion_cl(AVION_TABLE, cl);

        cd_total = Avion_fila.c_d + cd0;
        lift = 0.5 * ro * S_ref * cl * v_safe_sq;
        drag = 0.5 * ro * S_ref * cd_total * v_safe_sq + 0.5 * (S_Banner) * ro * (cd_Banner) * v_safe_sq;
        
        thrust_vec = T_total * [ Thrust_N ; 0; 0 ];  
        drag_vec   = -drag * versor_a;               % Resistencia opuesta al viento relativo
        
        % Sustentación en Ejes Viento
        Y_body_dir = T_total * [0; 1; 0];           % Eje lateral (envergadura)
        L_dir      = cross(versor_a, Y_body_dir);    % Vector ortogonal
        L_norm     = norm(L_dir);
        if L_norm > 1e-6
            L_dir = L_dir / L_norm;
        else
            L_dir = T_total * [0; 0; 1];
        end
        lift_vec   = lift * L_dir;                   % Vector de sustentación final
        
        % Dinámica aceleración inercial
        a = [0; 0; -g] + (thrust_vec + lift_vec + drag_vec) / MTOW;
        
        % Rumbo (Yaw)
        vx2_vy2 = V_curr(1)^2 + V_curr(2)^2;
        safe_den = max(vx2_vy2, 1e-12); 
        dY = (V_curr(1)*a(2) - V_curr(2)*a(1)) / safe_den;
        
        % Salidas para RK4
        dP = [V_curr(1); V_curr(2); V_curr(3)];
        dV = a;
        dE = Corriente_real/3600;
        
        % Guardamos datos extras
        extra.cl = cl;
        extra.cd_total = cd_total;
        extra.drag = drag;
        extra.Thrust_N = Thrust_N;
        extra.lift = lift;
        extra.a = a;
        extra.T_total = T_total;
        extra.V_versor = versor_a;
        extra.thrust_vec = thrust_vec;
        extra.drag_vec = drag_vec;
        extra.lift_vec = lift_vec;
        extra.Corriente_real = Corriente_real;
        extra.v_w = v_w_i; % Pasamos el viento real calculado
        extra.omega = omega_eq;
        extra.I_motor = I_motor;
        extra.I_batt = I_batt;
        extra.Vocv = Vocv;
    end

end

function Vocv = battery_cntm(Ah_consumidos,ncells,Q,E0,K,A,B)
%Función Consumo Batería (Disminuye por vuelta la efectividad)
it = min(max(Ah_consumidos,0), Q*0.999); % Evita Q-it igual a 0 o negativo
Vocv_cell = E0 - K*(Q/(Q-it))*it + A*exp(-B*it); % Modelo de tensión a lo largo del tiempo
Vocv = ncells*Vocv_cell; % Tension total
end

function omega_eq = motor_prop_eq(throttle_real, Vocv, v_air, PROP_TABLE,Kt,Ke,Rint,I0,Rbat,omega_seed)
%Halla el omega correcto que respeta el equilibrio
f = @(w) torque_motor(w,throttle_real,Vocv,Rbat,Kt,Ke,Rint,I0) - prop_torque(w, v_air, PROP_TABLE);

opts = optimset('Display','off','TolX',1e-6);
    try
        omega_eq = fzero(f, max(omega_seed, 1), opts);
        
    catch
        % Si fzero no logra acotar la raíz (ej. arranque desde reposo con
        % semilla mala), probamos con un barrido grueso para darle un
        % intervalo con cambio de signo antes de tirar la toalla.
        w_grid = linspace(1, 3000, 60); % rad/s, ajustar rango al tuyo (RPM_max*2*pi/60)
        f_grid = arrayfun(f, w_grid);
        idx = find(sign(f_grid(1:end-1)) ~= sign(f_grid(2:end)), 1, 'first');
        if isempty(idx)
            omega_eq = 0; % no hay equilibrio con empuje positivo (ESC apagado/trabado)
        else
            omega_eq = fzero(f, [w_grid(idx), w_grid(idx+1)], opts);
        end
    end
    omega_eq = max(omega_eq, 0);
end
function T = torque_motor(omega,throttle_real,Vocv,Rbat,Kt,Ke,Rint,I0)
I_motor = (throttle_real*Vocv - Ke*omega)/(Rint + throttle_real^2*Rbat);
I_motor = max(I_motor, 0);
T = Kt*(I_motor - I0);
end
function T = prop_torque(omega, v_air, PROP_TABLE)
    RPM = omega * 60/(2*pi);
    Prop_fila = interpProp(PROP_TABLE, v_air, RPM);
    T = abs(Prop_fila.Torque_Nm);
end