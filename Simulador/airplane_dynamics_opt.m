function [x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,inform,Energy,t] = airplane_dynamics_opt(MTOW,t_n,ro,S_ref,x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,throttle,cd0, ...
                                                                                          PROP_TABLE, MOTOR_TABLE,AVION_TABLE, ...
                                                                                          inform,Energy,t,S_Banner,cd_Banner)

    % --- CONFIGURACIÓN DE VECTORES DE ESTADO PARA RK4 ---
    P = [x; y; z];
    V = [v_x; v_y; v_z];
    Y = yaw_rad;
    E = Energy;
    
    % Inicialización del estado caótico de Lorenz (Viento)
    persistent C_caos;
    if isempty(C_caos)
        C_caos = [1.0; 1.0; 20.0]; 
    end

    persistent omega_seed;
    if isempty(omega_seed)
        omega_seed = 300; % rad/s, valor de arranque genérico, ajustalo si hace falta
    end

    % --- PASO 1: k1 ---
    [dP1, dV1, dY1, dE1, dC1, ~] = compute_derivatives(P, V, Y, C_caos);

    % --- PASO 2: k2 ---
    P2 = P + 0.5 * t_n * dP1;
    V2 = V + 0.5 * t_n * dV1;
    Y2 = Y + 0.5 * t_n * dY1;
    C2 = C_caos + 0.5 * t_n * dC1;
    [dP2, dV2, dY2, dE2, dC2, ~] = compute_derivatives(P2, V2, Y2, C2);

    % --- PASO 3: k3 ---
    P3 = P + 0.5 * t_n * dP2;
    V3 = V + 0.5 * t_n * dV2;
    Y3 = Y + 0.5 * t_n * dY2;
    C3 = C_caos + 0.5 * t_n * dC2;
    [dP3, dV3, dY3, dE3, dC3, ~] = compute_derivatives(P3, V3, Y3, C3);

    % --- PASO 4: k4 ---
    P4 = P + t_n * dP3;
    V4 = V + t_n * dV3;
    Y4 = Y + t_n * dY3;
    C4 = C_caos + t_n * dC3;
    [dP4, dV4, dY4, dE4, dC4, ~] = compute_derivatives(P4, V4, Y4, C4);

    % --- INTEGRACIÓN FINAL (PROMEDIO PONDERADO RK4) ---
    P = P + (t_n / 6) * (dP1 + 2*dP2 + 2*dP3 + dP4);
    V = V + (t_n / 6) * (dV1 + 2*dV2 + 2*dV3 + dV4);
    Y = Y + (t_n / 6) * (dY1 + 2*dY2 + 2*dY3 + dY4);
    E = E + (t_n / 6) * (dE1 + 2*dE2 + 2*dE3 + dE4);
    C_caos = C_caos + (t_n / 6) * (dC1 + 2*dC2 + 2*dC3 + dC4);

    % Variables de salida
    x = P(1); y = P(2); z = P(3);
    v_x = V(1); v_y = V(2); v_z = V(3);
    yaw_rad = Y;
    Energy = E;
    t = t + t_n;

    % Registro final de variables
    [~, ~, ~, ~, ~, extra] = compute_derivatives(P, V, Y, C_caos);


    % --- LOGGEO EN LA MATRIZ INFORM ---
    % Agregamos extra.E_wind(1) y extra.E_wind(2) en las filas 25 y 26
    inform = [inform [x; y; z; v_x; v_y; v_z; extra.a(1); extra.a(2); extra.a(3); t; extra.cl; extra.cd_total; extra.drag; extra.Thrust_N; ...
                 extra.lift_vec(1); extra.lift_vec(2); extra.lift_vec(3); ...
                 extra.thrust_vec(1); extra.thrust_vec(2); extra.thrust_vec(3); ...
                 extra.drag_vec(1); extra.drag_vec(2); extra.drag_vec(3); ...
                 extra.Corriente_real; ...
                 extra.E_wind(1); extra.E_wind(2); ...
                 roll_rad]];


%% =====================================================================
    %% FUNCIÓN ANIDADA: CÁLCULO DE DERIVADAS TRIDIMENSIONALES + CONFIG VIENTO
    %% =====================================================================
    function [dP, dV, dY, dE, dC, extra] = compute_derivatives(P_curr, V_curr, Y_curr, C_curr)
        
        % -----------------------------------------------------------------
        % CONTROL DE VIENTO: Elegí entre: 'caotico', 'constante' o 'desactivado'
        % -----------------------------------------------------------------
        modo_viento = 'constante'; 
        
        switch modo_viento
            case 'caotico'
                % Modelo de Wichita, Kansas dinámico (Lorenz suavizado)
                dC = lorenz_derivatives(C_curr) * 0.1; 
                viento_caotico_lineal = [C_curr(1); C_curr(2); 0.0] * 0.05;
                E_wind = [4; 0; 0.0] + viento_caotico_lineal; 
                B_wind = [0.0; 0.0; C_curr(3) * 0.0005]; 
                q_air = 0.15;
                
                % Limitador estricto a 30 km/h (8.33 m/s)
                viento_max_ms = 30 / 3.6;
                mag_viento = sqrt(E_wind(1)^2 + E_wind(2)^2);
                if mag_viento > viento_max_ms
                    E_wind = E_wind * (viento_max_ms / mag_viento);
                end
                
            case 'constante'
                % Viento fijo sin perturbaciones (Ideal para calibrar el guiado)
                dC = zeros(3,1); % Bloqueamos el avance de Lorenz
                E_wind = [5; 0; 0.0]; % Soplido fijo (~21 km/h totales constantes)
                B_wind = [0.0; 0.0; 0.0]; % Sin remolinos ni cizalladura lateral
                q_air = 0.0;
                
            case 'desactivado'
                % Atmósfera estándar en calma total (Viento cero)
                dC = zeros(3,1);
                E_wind = [0.0; 0.0; 0.0];
                B_wind = [0.0; 0.0; 0.0];
                q_air = 0.0;
        end
        % -----------------------------------------------------------------
        
        % --- A partir de acá abajo continúa la física normal de tu avión ---
        cos_p=cos(pitch_rad); sin_p=sin(pitch_rad);
        T_pitch=[cos_p 0 -sin_p; 0 1 0; sin_p 0 cos_p];
        cos_r=cos(roll_rad); sin_r=sin(roll_rad);
        T_roll=[1 0 0; 0 cos_r -sin_r; 0 sin_r cos_r];
        cos_y=cos(Y_curr); sin_y=sin(Y_curr);
        T_yaw=[cos_y -sin_y 0; sin_y cos_y 0; 0 0 1];
        
        T_total = T_yaw * T_pitch * T_roll;
        
        % --- VELOCIDADES RELATIVAS (El viento se resta en el plano horizontal) ---
        V_rel = [V_curr(1); V_curr(2); V_curr(3)] - E_wind;
        v_rel_mag = sqrt(V_rel(1)^2 + V_rel(2)^2 + V_rel(3)^2);
        v_safe = max(v_rel_mag, 1e-6);
        
        V_versor = V_rel / v_safe;
        v_normal_helice = abs(dot(T_total * [1;0;0], V_rel));
        
        % Fuerza de Lorentz (Efecto de cizalladura lateral en X-Y)
        F_lorentz_wind = q_air * cross([V_curr(1); V_curr(2); V_curr(3)], B_wind);
        
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
        CL_max = 0.62;   % Ajustar según el CLmax real de tu polar
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
        drag_vec   = -drag * V_versor;               % Resistencia opuesta al viento relativo
        
        % Sustentación en Ejes Viento
        Y_body_dir = T_total * [0; 1; 0];           % Eje lateral (envergadura)
        L_dir      = cross(V_versor, Y_body_dir);    % Vector ortogonal
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
        extra.V_versor = V_versor;
        extra.thrust_vec = thrust_vec;
        extra.drag_vec = drag_vec;
        extra.lift_vec = lift_vec;
        extra.Corriente_real = Corriente_real;
        extra.E_wind = E_wind; % Pasamos el viento real calculado
        extra.omega = omega_eq;
        extra.I_motor = I_motor;
        extra.I_batt = I_batt;
        extra.Vocv = Vocv;
    end

    function dC = lorenz_derivatives(C_curr)
        sigma = 10.0; rho = 28.0; beta = 8/3;
        dC = zeros(3,1);
        dC(1) = sigma * (C_curr(2) - C_curr(1));
        dC(2) = C_curr(1) * (rho - C_curr(3)) - C_curr(2);
        dC(3) = C_curr(1) * C_curr(2) - beta * C_curr(3);
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