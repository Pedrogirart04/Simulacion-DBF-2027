%% base_simulator.m
% Simulador base DBF — Aero ITBA 2027
% Copiar este archivo y renombrar para cada misión (ej: Sim_M1.m)

clc; clear; close all;
tic
%% ========================================================================
%  SECCIÓN 1: CONFIGURACIÓN DE MISIÓN
%  ========================================================================

% --- Avión ---
MTOW     = 8;           % [kg] Masa total de despegue
S_ref    = 1.386;        % [m²] Superficie alar de referencia
b        = 2.5;          % [m]  Envergadura (para reportes)
cd0      = 0.05;        % [-]  Factor de Corrección Drag (Ver con Mati)
CL_max   = 0.62;         % [-]  CL máximo de la polar (para warning de stall)
crud     = 1;            % [-]  Factor de conservadurismo de drag: drag = drag*crud 

% --- Propulsión ---
prop_file  = 'PER3_20x10E.dat';
motor_file = 'Scorpion A-5025-310kv.dat';
polar_file = 'CONDOR.dat';

% --- Condiciones de vuelo ---
rho      = 1.225;        % [kg/m³] Densidad del aire (ISA nivel del mar)
V_inicio = 30;           % [m/s]   Velocidad inicial (Arranque ya en vuelo)
throttle = 1643;         % [μs]    Señal PWM al ESC (Valor de Referencia)

% --- Viento (default global; los tramos pueden sobreescribir) ---
wind_steady_default = [5; -2; 0];   % [m/s] viento estacionario en ejes inerciales (x,y,z)
turbulencia_default  = 'none';     % 'none' | 'light' | 'moderate'

% --- Control de tramos por variable de referencia ---
Kp_throttle       = 30;    % [us / (m/s)]     ganancia proporcional del PI de velocidad
Ki_throttle       = 15;     % [us / (m/s)/s]   ganancia integral del PI de velocidad
throttle_rate_max = 1000;   % [us/s]           tasa máxima de cambio de throttle (todos los modos)


% --- Despegue ---
despegue_on         = true;   % on/off: si es false, arranca directo en vuelo (como antes)
mu_rodadura         = 0.05;   % [-] fricción de rodadura en pista
k_rot               = 1.4;   % [-] margen de V_takeoff sobre V_stall
CL_suelo            = 0;      % [-] CL supuesto en actitud de pista
gamma_obj_deg       = 12;     % [deg] ángulo de trayectoria objetivo del ascenso
z_objetivo_despegue = 30;     % [m] altura de nivelación (arranca crucero)
throttle_despegue   = 1900;   % [us] throttle fijo durante todo el despegue

% --- Batería: warnings de SoC + aborto de misión ---
Q_bateria    = 3.3;             % [Ah] 
soc_umbrales = [75 50 30 20 15 10];   % [%] umbrales de aviso; en el más bajo (<=10%) se aborta la misión

% =========================================================================
% CIRCUITO Y MISIÓN (INDIVIDUAL POR TRAMO)

n_vueltas    = 1;     % Cantidad de vueltas
t_transicion = 1.0;   % [s] Tiempo para maniobra de rolido (0 a bank)
V_min_offset = 3;     % [m/s] piso de seguridad para heading_offset = C/V (evita dividir por V chica)

circuito = [
    % --- TRAMO 1: Recta ---
    struct('tipo', 'recta', 'largo_m', 100, 'delta_yaw_deg', 0, 'bank_deg', 0, ...
           'modo_control', 'throttle', 'valor_ref', 1954, ...
           'fase_cd0', 'crucero','wind_steady', [5; 0;0], 'turbulencia', 'moderate'), ...

    % --- TRAMO 2: Giro
    struct('tipo', 'giro', 'largo_m', 0, 'delta_yaw_deg', 180, 'bank_deg', 60, ...
           'modo_control', 'CL', 'valor_ref', CL_max*0.9-0.1, ...
           'fase_cd0', 'crucero','wind_steady', [0; 0; 0], 'turbulencia', 'none'), ...

    % --- TRAMO 3: Recta (V cte)
    struct('tipo', 'recta', 'largo_m', 100, 'delta_yaw_deg', 0, 'bank_deg', 0, ...
           'modo_control', 'throttle', 'valor_ref', 1954, ...
            'fase_cd0','crucero','wind_steady', [5; 0; 0], 'turbulencia', 'moderate'), ...

    % --- TRAMO 4: Giro
    struct('tipo', 'giro', 'largo_m', 0, 'delta_yaw_deg', 180, 'bank_deg', 60, ...
           'modo_control', 'CL', 'valor_ref', CL_max*0.9-0.1, ...
           'fase_cd0', 'crucero','wind_steady', [0; 0; 0], 'turbulencia', 'none'), ...

    % --- TRAMO 5: Recta ---
    %struct('tipo', 'recta', 'largo_m', 90, 'delta_yaw_deg', 0, 'bank_deg', 0, ...
    %       'modo_control', 'throttle', 'valor_ref', 1954, 'heading_offset', 0, 'fase_cd0', 'crucero'), ...

    % --- TRAMO 6: Giro 360
    %struct('tipo', 'giro', 'largo_m', 0, 'delta_yaw_deg', 360, 'bank_deg', 60, ...
    %       'modo_control', 'throttle', 'valor_ref', 1954, 'heading_offset', 13, 'fase_cd0', 'crucero'), ...

];

% Estado inicial de la velocidad angular del motor
omega = 300; % [rad/s]

% Ángulo de alabeo máximo para referencia
bank_angle = max([circuito.bank_deg]);
% =========================================================================

% --- Hecho para Banner, Modificable para sensor ---
% --- Banner (poner 0 si no hay) ---
S_Banner  = 0;           % [m²] Superficie del banner
cd_Banner = 0;           % [-]  CD del banner

% --- Parametros de Simulación ---
dt       = 0.1;         % [s] Paso de tiempo del integrador
t_max    = 300;          % [s] Tiempo máximo de misión (5 min)

% --- Scoring (ajustar según misión) ---
% Dejar vacío si la misión no tiene scoring propio
scoring_type = 'none';   % 'none', 'M1', 'M2', 'M3'

% Parámetros específicos de scoring (solo si aplica):
% n_pasajeros = 151;
% n_cargo = 4;
% l_banner = 3;     % [m]



%% ========================================================================
%  SECCIÓN 2: CARGA DE DATOS
%  ========================================================================
% Configurar paths relativos al repositorio
repo_root = fileparts(mfilename('fullpath'));  % carpeta donde está ESTE script
% Si el script está en una subcarpeta, subir un nivel:
% repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repo_root, 'simulador'));
addpath(fullfile(repo_root, 'datos', 'helices'));
addpath(fullfile(repo_root, 'datos', 'motores'));
addpath(fullfile(repo_root, 'datos', 'polares'));
repo_root = fileparts(fileparts(mfilename('fullpath')));  % sube de simulador/ a raíz

%Si el archivo es M1 desde misiones/ descomentar la linea de abajo
%repo_root = fileparts(fileparts(mfilename('fullpath')));  % sube de misiones/ a raíz
%Borrar el otro que sube de simulador/ a raiz

fprintf('Cargando datos...\n');

% Tabla de hélice (formato APC → tabla con RPM, V_ms, Ct, Cp, Thrust_N, etc.)
PROP_TABLE = prop(prop_file);                    % ← prop.m

% Tabla de motor (formato .dat → tabla con ESC_throttle, RPM, current, torque, etc.)
MOTOR_TABLE = leer_dat_mot(motor_file);          % ← leer_dat_mot.m
MOTOR_TABLE.torque = abs(MOTOR_TABLE.torque);    % Forzar torques positivos

% Tabla de polar del avión (formato .dat → tabla con cl, cd)
AVION_TABLE = leer_dat_avion(polar_file);        % ← leer_dat_avion.m

fprintf('  Hélice:  %s (%d puntos)\n', prop_file, height(PROP_TABLE));
fprintf('  Motor:   %s (%d puntos)\n', motor_file, height(MOTOR_TABLE));
fprintf('  Polar:   %s (%d puntos, CL_max_tabla = %.3f)\n', ...
        polar_file, height(AVION_TABLE), max(AVION_TABLE.cl));
fprintf('Datos cargados.\n\n');



%% ========================================================================
%  SECCIÓN 3: CONDICIONES INICIALES
%  ========================================================================

% --- Log de datos ---
t_max_est = 600; % Estimación de tiempo máximo de vuelo [s]
max_pasos = ceil(t_max_est / dt) + 2000;
inform = zeros(33, max_pasos);
step_idx = 0;

% --- Estado del avión ---
t = 0;
x = 0;    y = 0;    z = 0;
v_x = V_inicio;     v_y = 0;    v_z = 0;
pitch_rad = 0;
roll_rad  = 0;
roll_rate = 0; 
yaw_rad   = 0;

% --- Energía ---
Energy = 0;          % [Ah] consumidos (acumulador)

% --- Contadores de misión ---
vueltas_completadas = 0;
t_por_vuelta = zeros(1, n_vueltas);   % Guardar tiempo de cada vuelta

mision_abortada = false; %Por si se quiere abortar la misión

% --- Estado de avisos de batería (SoC) ---
soc_avisado   = false(size(soc_umbrales));   % marca qué umbrales ya avisamos (una vez cada uno)
t_soc_avisos  = NaN(size(soc_umbrales));     % tiempo [s] en que se cruzó cada umbral (NaN si no se cruzó)

% --- Pre-cálculos del circuito ---
% Convertir ángulos a radianes una sola vez
bank_rad = deg2rad(bank_angle);

% --- Parámetros del filtro de 2do orden (banqueo, crítico) ---
wn             = 5.8 / t_transicion;   % frecuencia natural [rad/s] (t_transicion ~ tiempo de establecimiento al 2%)
max_roll_rate  = 3 * wn * bank_rad;    % tope de seguridad generoso, no debería activarse en operación normal
max_pitch_rate = 3 * wn * deg2rad(gamma_obj_deg);   % tope de seguridad análogo, para el pitch del despegue

% --- Estado del controlador de throttle (para tramos con V o CL de referencia) ---
throttle_ctrl      = throttle;   % arranca en el throttle inicial de la config
e_prev             = 0;
modo_control_prev  = '';

%V_stall (Referencia de despegue)
g = 9.81;
V_stall   = sqrt(2*MTOW*g / (rho*S_ref*CL_max));


fprintf('=== INICIO DE SIMULACIÓN ===\n');
fprintf('MTOW = %.1f kg | V_ini = %.1f m/s | Bank = %.0f°\n', MTOW, V_inicio, bank_angle);
fprintf('V_stall= %.1f m/s',V_stall)
% Extrae las rectas configuradas en el circuito
% rectas = circuito(strcmp({circuito.tipo}, 'recta'));
% fprintf('Circuito: piernas de %d y %d m | %d vueltas objetivo\n', ...
%         rectas(1).largo_m, rectas(2).largo_m, n_vueltas);
% fprintf('dt = %.3f s | t_max = %.0f s\n\n', dt, t_max);


%% ========================================================================
%  SECCIÓN 4: DESPEGUE
%  ========================================================================
if despegue_on
    [x, y, z, v_x, v_y, v_z, pitch_rad, yaw_rad, t, Energy, inform_despegue, n_pasos_despegue] = ...
                simular_despegue(MTOW, dt, rho, S_ref, cd0, CL_max, CL_suelo, mu_rodadura, k_rot, ...
                          gamma_obj_deg, z_objetivo_despegue, throttle_despegue, ...
                          PROP_TABLE, MOTOR_TABLE, AVION_TABLE, S_Banner, cd_Banner, ...
                          wn, max_pitch_rate, crud, Q_bateria);

    inform(:, 1:n_pasos_despegue) = inform_despegue;
    step_idx = n_pasos_despegue;
    fprintf('Despegue: %.1f m de pista+ascenso, %.1f s, altura final %.1f m, V final %.1f m/s\n\n', ...
            x, t, z, sqrt(v_x^2+v_y^2+v_z^2));
else
    fprintf('Despegue: SALTADO (arranca en vuelo a %.1f m/s)\n\n', V_inicio);
end


%% =========================================================================
% SECCIÓN 5: BUCLE PRINCIPAL DE NAVEGACIÓN (UNIFIED STATE MACHINE)
% =========================================================================

for vuelta = 1:n_vueltas
        t_inicio_vuelta = t;
    for t_idx = 1:length(circuito)
        
        tramo = circuito(t_idx);
        if isfield(tramo, 'modo_control')
            modo_control = tramo.modo_control;
            valor_ref    = tramo.valor_ref;
        else
            modo_control = 'throttle';
            valor_ref    = tramo.throttle;
        end
        
        if isfield(tramo, 'wind_steady')
            wind_steady_tramo = tramo.wind_steady;
        else
            wind_steady_tramo = wind_steady_default;
        end
        if isfield(tramo, 'turbulencia')
            turbulencia_tramo = tramo.turbulencia;
        else
            turbulencia_tramo = turbulencia_default;
        end
        
         % --- Offset de anticipación de salida de giro (heading_offset) ---
        if strcmp(tramo.tipo, 'giro')
            if isfield(tramo, 'heading_offset') && ~isempty(tramo.heading_offset)
                heading_offset_manual = deg2rad(tramo.heading_offset);
                usar_offset_manual    = true;
            else
                C_turn = calc_offset_giro(deg2rad(tramo.bank_deg), wn, dt);
                usar_offset_manual  = false;
            end
        end

        % Registro de punto inicial del tramo
        x_start = x; 
        y_start = y;
        yaw_acum = 0;
        
        t_inicio_pierna = t;

        % --- IMPRIMIR PROGRESO EN CONSOLA ---
        if strcmp(tramo.tipo, 'recta')
            fprintf('Vuelta %d/%d | Tramo %d: RECTA (%d m)\n', ...
                vuelta, n_vueltas, t_idx, tramo.largo_m);
        elseif strcmp(tramo.tipo, 'giro')
            fprintf('Vuelta %d/%d | Tramo %d: GIRO (%d° a bank %d°)\n', ...
                vuelta, n_vueltas, t_idx, tramo.delta_yaw_deg, tramo.bank_deg);
        end
        
        en_tramo = true;
        
        while en_tramo
            yaw_prev = yaw_rad;
            V_inst   = sqrt(v_x^2 + v_y^2 + v_z^2);   
            
            % --- 1. CÁLCULO DEL ALABEO(YAW) OBJETIVO ---
            if strcmp(tramo.tipo, 'recta')
                target_bank = 0;
            elseif strcmp(tramo.tipo, 'giro')
                target_bank = sign(tramo.delta_yaw_deg) * abs(deg2rad(tramo.bank_deg));
            end
            
            % --- 2. FILTRO DE 2DO ORDEN (CRÍTICAMENTE AMORTIGUADO) PARA EL BANQUEO ---
            e0 = roll_rad - target_bank;
            v0 = roll_rate;
            exp_term = exp(-wn*dt);
            e_new = (e0 + (v0 + wn*e0)*dt) * exp_term;
            v_new = (v0 - wn*(v0 + wn*e0)*dt) * exp_term;
            
            roll_rate = sign(v_new) * min(abs(v_new), max_roll_rate);
            roll_rad  = target_bank + e_new;
            
            % --- 3. EVALUAR CONDICIONES DE FIN DE TRAMO ---
            if strcmp(tramo.tipo, 'recta')
                dist_recorrida = norm([x - x_start, y - y_start]);
                if dist_recorrida >= tramo.largo_m
                    en_tramo = false;
                    break;
                end
                
             elseif strcmp(tramo.tipo, 'giro')
                    % Offset de anticipación: manual si está definido en el tramo, si no C_turn/V (automático)
                    if usar_offset_manual
                        heading_offset_rad = heading_offset_manual;
                    else
                        heading_offset_rad = C_turn / max(V_inst, V_min_offset);
                    end
                    target_yaw_rad = abs(deg2rad(tramo.delta_yaw_deg)) - heading_offset_rad;
                    if yaw_acum >= target_yaw_rad
                        en_tramo = false;
                        break;
                    end
             end
            
            dt_pierna = t - t_inicio_pierna;
            cd0_actual = case_cd0(tramo.fase_cd0, dt_pierna, V_inst, [], cd0);

       % --- CONTROL DE THROTTLE SEGÚN VARIABLE DE REFERENCIA DEL TRAMO ---
             switch modo_control
                case 'throttle'
                    throttle_obj = valor_ref;
                    V_obj = NaN;  

                case 'V'
                    V_obj = valor_ref;
                    if ~strcmp(modo_control_prev,'V') && ~strcmp(modo_control_prev,'CL')
                        e_prev = V_obj - V_inst;  
                    end
                    e = V_obj - V_inst;
                    throttle_obj = throttle_ctrl + Kp_throttle*(e - e_prev) + Ki_throttle*e*dt;
                    e_prev = e;

                case 'CL'
                    V_obj = sqrt(2*MTOW*9.81 / (rho*S_ref*cos(roll_rad)*cos(pitch_rad)*valor_ref));
                    if ~strcmp(modo_control_prev,'V') && ~strcmp(modo_control_prev,'CL')
                        e_prev = V_obj - V_inst;
                    end
                    e = V_obj - V_inst;
                    throttle_obj = throttle_ctrl + Kp_throttle*(e - e_prev) + Ki_throttle*e*dt;
                    e_prev = e;
            end
            modo_control_prev = modo_control;

            switch modo_control
                case 'throttle', modo_code = 0;
                case 'V',        modo_code = 1;
                case 'CL',       modo_code = 2;
            end

            % --- LIMITADOR DE TASA (mismo criterio que el banqueo) + CLAMP FÍSICO ---
            d_thr = throttle_obj - throttle_ctrl;
            throttle_ctrl = throttle_ctrl + sign(d_thr) * min(abs(d_thr), throttle_rate_max*dt);
            throttle_ctrl = min(max(throttle_ctrl, 1225), 2000);

            throttle = throttle_ctrl;


           % --- 4. INTEGRACIÓN FÍSICA Y DINÁMICA (FIRMA ACTUAL SIN OMEGA) ---
            step_idx = step_idx + 1;
            
            [x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad, log_step, Energy, t] = ...
                airplane_dynamics_opt(MTOW, dt, rho, S_ref, ...
                    x, y, z, v_x, v_y, v_z, roll_rad, pitch_rad, yaw_rad, ...
                    throttle, cd0_actual, ...
                    PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                    Energy, t, S_Banner, cd_Banner, CL_max, ...
                    V_inst, wind_steady_tramo, turbulencia_tramo,crud, Q_bateria);
                            
            % --- 5. ALMACENAMIENTO EN LOG PREASIGNADO ---
            inform(:, step_idx) = [log_step; modo_code; V_obj; throttle_ctrl; pitch_rad];
            
            % --- 6. CONTROL DE ESTADO DE CARGA DE BATERÍA (AVISOS + ABORTO) ---
            soc_pct = 100 * (1 - Energy/Q_bateria);
            for k_soc = 1:length(soc_umbrales)
                if ~soc_avisado(k_soc) && soc_pct <= soc_umbrales(k_soc)
                    soc_avisado(k_soc)  = true;
                    t_soc_avisos(k_soc) = t;
                    if soc_umbrales(k_soc) > 10
                        fprintf('  >> AVISO BATERÍA: SoC = %.0f%% (umbral %d%%) en t = %.1f s\n', ...
                                soc_pct, soc_umbrales(k_soc), t);
                    else
                        fprintf('  >> CORTE DE MISIÓN: SoC = %.0f%% <= %d%% en t = %.1f s. Abortando.\n', ...
                                soc_pct, soc_umbrales(k_soc), t);
                        mision_abortada = true;
                    end
                end
            end

            % --- 7. CÁLCULO DE ÁNGULO GIRADO ACUMULADO ---
            dyaw = yaw_rad - yaw_prev;
            if dyaw > pi,  dyaw = dyaw - 2*pi; end
            if dyaw < -pi, dyaw = dyaw + 2*pi; end
            yaw_acum = yaw_acum + abs(dyaw);

            if mision_abortada
                break;
            end
            
        end % while en_tramo
        if mision_abortada
            break;
        end
    end % for tramo
    if mision_abortada
        break;
    end
    t_por_vuelta(vuelta) = t - t_inicio_vuelta;
    vueltas_completadas = vueltas_completadas + 1;
end % for vuelta

% Recorte final de la matriz de datos al número exacto de pasos
inform = inform(:, 1:step_idx);

%% ========================================================================
%  SECCIÓN 6: RESULTADOS Y SCORING
%  ========================================================================

% --- Resumen de vuelo ---
fprintf('============================================\n');
fprintf('         RESUMEN DE VUELO\n');
fprintf('============================================\n');

if mision_abortada
    fprintf(' *** MISIÓN ABORTADA POR BATERÍA BAJA (SoC <= %d%%) ***\n', min(soc_umbrales));
end

fprintf(' Avión:            %s\n', polar_file);
fprintf(' Motor:            %s\n', motor_file);
fprintf(' Hélice:           %s\n', prop_file);
fprintf(' MTOW:             %.1f kg\n', MTOW);
fprintf(' Throttle:         %d μs\n', throttle);
fprintf(' Bank angle:       %.0f°\n', bank_angle);
fprintf('--------------------------------------------\n');
fprintf(' Vueltas completas:  %d / %d\n', vueltas_completadas, n_vueltas);
fprintf(' Tiempo total:       %.2f s (%.1f min)\n', t, t/60);
fprintf(' Energía consumida:  %.3f Ah\n', Energy);

if vueltas_completadas > 0
    fprintf('--------------------------------------------\n');
    fprintf(' Tiempo por vuelta:\n');
    for k = 1:vueltas_completadas
        fprintf('   Vuelta %d:  %.2f s\n', k, t_por_vuelta(k));
    end
    t_promedio = mean(t_por_vuelta(1:vueltas_completadas));
    fprintf('   Promedio:  %.2f s\n', t_promedio);
    fprintf('--------------------------------------------\n');

    % Estimar vueltas en el tiempo de misión
    % (usando el tiempo promedio, asumiendo batería suficiente)
    vueltas_estimadas = floor(t_max / t_promedio);
    fprintf(' Vueltas estimadas en %.0f s: %d\n', t_max, vueltas_estimadas);
    fprintf(' Energía estimada para %d vueltas: %.3f Ah\n', ...
            vueltas_estimadas, Energy / vueltas_completadas * vueltas_estimadas);
end

% Velocidad promedio del log (si hay datos)
if ~isempty(inform)
    v_mag = sqrt(inform(4,:).^2 + inform(5,:).^2 + inform(6,:).^2);
    fprintf('--------------------------------------------\n');
    fprintf(' Velocidad media:    %.1f m/s (%.1f km/h)\n', mean(v_mag), mean(v_mag)*3.6);
    fprintf(' Velocidad máx:      %.1f m/s (%.1f km/h)\n', max(v_mag), max(v_mag)*3.6);
    fprintf(' Velocidad mín:      %.1f m/s (%.1f km/h)\n', min(v_mag), min(v_mag)*3.6);
    fprintf(' CL máx alcanzado:   %.3f (CL_max_data = %.3f)\n', max(inform(11,:)), CL_max);
    fprintf(' Corriente media:    %.1f A\n', mean(inform(24,:)));
    fprintf(' Corriente máx:      %.1f A\n', max(inform(24,:)));
end
fprintf('============================================\n\n');

% --- Scoring por misión ---
% --- Adaptar a Nueva Competencia (Sensor) ---
switch scoring_type
    case 'M1'
        % ==== MISIÓN 1 ====
        % Ajustar las fórmulas según las reglas DBF 2027
        fprintf('============================================\n');
        fprintf('       SCORING MISIÓN 1\n');
        fprintf('============================================\n');
        fprintf(' (Completar con reglas 2027)\n');
        fprintf('============================================\n\n');

    case 'M2'
        % ==== MISIÓN 2:  ====
        % Ajustar las fórmulas según las reglas DBF 2027

    case 'M3'
        % ==== MISIÓN 3:  ====

    case 'none'
        % Sin scoring — solo el resumen de vuelo de arriba

    otherwise
        warning('scoring_type "%s" no reconocido. Opciones: none, M1, M2, M3', scoring_type);
end

disp("----- \n")
disp("Tiempo de Simulacion \n")
toc
disp("----- \n")

%% ========================================================================
%  SECCIÓN 7: GRÁFICOS
%  ========================================================================

% --- Configuración de gráficos ---
plot1_on = true;   % Trayectorias 2D/3D
plot2_on = true;   % Panel de variables vs tiempo
plot_modo_control = true;   % on/off: overlay V_obj sobre V_ms + panel de modo activo

panels = {
    {'V_ms','Airspeed_ms'}
    {'Altitude_m'}
    {'Throttle_us'}
    {'Thrust_N','Drag_N'}
    {{'Corriente_A'}, {'Energia_Ah'}}
    {'Roll_deg'}
};

% Cada fila de panels = un subplot. Cada celda dentro = una variable.
% Nombres disponibles: V_ms, CL, CD, LD, Thrust_N, Drag_N, Corriente_A,
%                       Energia_Ah, Roll_deg, Altitude_m, RPM, Viento_x_ms,
%                       Viento_y_ms, Viento_z_ms, Airspeed_ms,
%                       modo_control_code, V_obj, Throttle_us
%                       
% Si se coloca doble llave es doble eje {{'Corriente_A'}, {'Energia_Ah'}}

plot_simulation(inform, t_por_vuelta, vueltas_completadas, ...
                MTOW, rho, S_ref, CL_max, t, ...
                plot1_on, plot2_on, panels, plot_modo_control, ...
                t_soc_avisos, soc_umbrales);