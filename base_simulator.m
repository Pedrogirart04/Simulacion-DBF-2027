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

% --- Propulsión ---
prop_file  = 'PER3_20x10E.dat';
motor_file = 'Scorpion A-5025-310kv.dat';
polar_file = 'CONDOR.dat';

% --- Condiciones de vuelo ---
rho      = 1.225;        % [kg/m³] Densidad del aire (ISA nivel del mar)
V_inicio = 20;           % [m/s]   Velocidad inicial (arranque ya en vuelo)
throttle = 1643;         % [μs]    Señal PWM al ESC (fijo por ahora)

% =========================================================================
% CIRCUITO Y MISIÓN (INDIVIDUAL POR TRAMO)

n_vueltas    = 1;     % Cantidad de vueltas
t_transicion = 1.0;   % [s] Tiempo para maniobra de rolido (0 a bank)

circuito = [
    % --- TRAMO 1: Recta ---
    struct('tipo', 'recta', 'largo_m', 100, 'delta_yaw_deg', 0, 'bank_deg', 0, 'throttle', 1954, 'heading_offset', 0, 'fase_cd0', 'crucero'), ...

    % --- TRAMO 2: Giro 
    struct('tipo', 'giro',  'largo_m', 0,   'delta_yaw_deg', 180,   'bank_deg', 60, 'throttle', 1954, 'heading_offset', 15.5, 'fase_cd0', 'crucero'), ...

    % --- TRAMO 3: Recta ---
    struct('tipo', 'recta', 'largo_m', 130,  'delta_yaw_deg', 0,    'bank_deg', 0,  'throttle', 1954, 'heading_offset', 0, 'fase_cd0', 'crucero'), ...

    % --- TRAMO 4: Giro 
    struct('tipo', 'giro',  'largo_m', 0,   'delta_yaw_deg', 180,  'bank_deg', 60, 'throttle', 2050, 'heading_offset',16, 'fase_cd0', 'crucero'), ...

    % --- TRAMO 5: Recta ---
    %struct('tipo', 'recta', 'largo_m', 90, 'delta_yaw_deg', 0,    'bank_deg', 0,  'throttle', 1954, 'heading_offset', 0, 'fase_cd0', 'crucero'), ...

    % --- TRAMO 6: Giro 360
    %struct('tipo', 'giro',  'largo_m', 0,   'delta_yaw_deg', 360,  'bank_deg', 60, 'throttle', 1954, 'heading_offset', 13, 'fase_cd0', 'crucero'), ...

    % --- TRAMO 6: Recta
    %struct('tipo', 'recta', 'largo_m', 5, 'delta_yaw_deg', 0,    'bank_deg', 0,  'throttle', 1954, 'heading_offset', 0, 'fase_cd0', 'crucero'), ...
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

% --- Simulación ---
dt       = 0.5;         % [s] Paso de tiempo del integrador
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
inform = zeros(29, max_pasos);
step_idx = 0;

% --- Estado del avión ---
t = 0;
x = 0;    y = 0;    z = 0;
v_x = V_inicio;     v_y = 0;    v_z = 0;
pitch_rad = 0;
roll_rad  = 0;
yaw_rad   = 0;

% --- Energía ---
Energy = 0;          % [Ah] consumidos (acumulador)

% --- Contadores de misión ---
vueltas_completadas = 0;
t_por_vuelta = zeros(1, n_vueltas);   % Guardar tiempo de cada vuelta

mision_abortada = false; %Por si se quiere abortar la misión

% --- Pre-cálculos del circuito ---
% Convertir ángulos a radianes una sola vez
bank_rad = deg2rad(bank_angle);


fprintf('=== INICIO DE SIMULACIÓN ===\n');
fprintf('MTOW = %.1f kg | V_ini = %.1f m/s | Bank = %.0f°\n', MTOW, V_inicio, bank_angle);
% Extrae las rectas configuradas en el circuito
rectas = circuito(strcmp({circuito.tipo}, 'recta'));
fprintf('Circuito: piernas de %d y %d m | %d vueltas objetivo\n', ...
        rectas(1).largo_m, rectas(2).largo_m, n_vueltas);
fprintf('dt = %.3f s | t_max = %.0f s\n\n', dt, t_max);


%% ========================================================================
%  SECCIÓN 4: DESPEGUE
%  ========================================================================
% TODO: Implementar modelo de despegue
fprintf('Despegue: SALTADO (arranca en vuelo a %.1f m/s)\n\n', V_inicio);


% =========================================================================
% SECCIÓN 5: BUCLE PRINCIPAL DE NAVEGACIÓN (UNIFIED STATE MACHINE)
% =========================================================================

for vuelta = 1:n_vueltas
    for t_idx = 1:length(circuito)
        
        tramo = circuito(t_idx);
        throttle = tramo.throttle;
        
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
            
            % --- 1. CÁLCULO DEL ALABEO(YAW) OBJETIVO Y TASA MÁXIMA DE ROLIDO ---
            if strcmp(tramo.tipo, 'recta')
                target_bank = 0;
            elseif strcmp(tramo.tipo, 'giro')
                target_bank = sign(tramo.delta_yaw_deg) * abs(deg2rad(tramo.bank_deg));
            end
            
            % Tasa de alabeo maxima permitida por segundo [rad/s]
            % Asumimos referencia relativa al bank maximo parametrizado
            max_roll_rate = max(abs(target_bank), deg2rad(30)) / t_transicion;
            
            % --- 2. APLICACIÓN DE RAMPA SUAVE (Entrada y Salida de Giros) ---
            d_roll = target_bank - roll_rad;
            roll_rad = roll_rad + sign(d_roll) * min(abs(d_roll), max_roll_rate * dt);
            
            % --- 3. EVALUAR CONDICIONES DE FIN DE TRAMO ---
            if strcmp(tramo.tipo, 'recta')
                dist_recorrida = norm([x - x_start, y - y_start]);
                if dist_recorrida >= tramo.largo_m
                    en_tramo = false;
                    break;
                end
                
            elseif strcmp(tramo.tipo, 'giro')
                % Condición con offset para iniciar el des-alabeo a tiempo
                target_yaw_rad = abs(deg2rad(tramo.delta_yaw_deg)) - deg2rad(tramo.heading_offset);
                if yaw_acum >= target_yaw_rad
                    en_tramo = false;
                    break;
                end
            end
            
            dt_pierna = t - t_inicio_pierna;
            V_inst = sqrt(v_x^2 + v_y^2 + v_z^2);
            cd0_actual = case_cd0(tramo.fase_cd0, dt_pierna, V_inst, [], cd0);

           % --- 4. INTEGRACIÓN FÍSICA Y DINÁMICA (FIRMA ACTUAL SIN OMEGA) ---
            step_idx = step_idx + 1;
            
            [x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad, log_step, Energy, t] = ...
                airplane_dynamics_opt(MTOW, dt, rho, S_ref, ...
                    x, y, z, v_x, v_y, v_z, roll_rad, pitch_rad, yaw_rad, ...
                    throttle, cd0_actual, ...
                    PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                    Energy, t, S_Banner, cd_Banner,CL_max);
                
            % --- 5. ALMACENAMIENTO EN LOG PREASIGNADO ---
            inform(:, step_idx) = log_step;
            
            % --- 6. CÁLCULO DE ÁNGULO GIRADO ACUMULADO ---
            dyaw = yaw_rad - yaw_prev;
            if dyaw > pi,  dyaw = dyaw - 2*pi; end
            if dyaw < -pi, dyaw = dyaw + 2*pi; end
            yaw_acum = yaw_acum + abs(dyaw);
            
        end % while en_tramo
    end % for tramo
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
        % ==== MISIÓN 2: Charter Flight ====
        % Constantes de scoring (Table 3.3.3.2 — ajustar a reglas 2027)
        Ip1 = 6;     % Ingreso fijo por pasajero
        Ip2 = 2;     % Ingreso por pasajero por vuelta
        Ic1 = 10;    % Ingreso fijo por cargo
        Ic2 = 8;     % Ingreso por cargo por vuelta
        Ce  = 10;    % Costo operativo base por vuelta
        Cp  = 0.5;   % Costo operativo por pasajero por vuelta
        Cc  = 2;     % Costo operativo por cargo por vuelta
        EF  = 1;     % Factor de eficiencia

        laps = vueltas_estimadas;
        Income = (n_pasajeros * (Ip1 + Ip2 * laps)) + ...
                 (n_cargo * (Ic1 + Ic2 * laps));
        Cost = laps * (Ce + n_pasajeros * Cp + n_cargo * Cc) * EF;
        Net_Income = Income - Cost;

        fprintf('============================================\n');
        fprintf('       SCORING MISIÓN 2: CHARTER FLIGHT\n');
        fprintf('============================================\n');
        fprintf(' Pasajeros:    %d\n', n_pasajeros);
        fprintf(' Cargo:        %d unidades\n', n_cargo);
        fprintf(' Vueltas:      %d (en %.0f s)\n', laps, t_max);
        fprintf('--------------------------------------------\n');
        fprintf(' Ingreso:      $ %.2f\n', Income);
        fprintf(' Costo:        $ %.2f\n', Cost);
        fprintf('--------------------------------------------\n');
        fprintf(' >> NET INCOME:  $ %.2f <<\n', Net_Income);
        fprintf('============================================\n\n');

    case 'M3'
        % ==== MISIÓN 3: Banner Flight ====
        m_to_ft = 3.28084;
        l_banner_ft = l_banner * m_to_ft;
        RAC = MTOW * 2.20462;   % kg a lbs como estimación de RAC

        laps = vueltas_estimadas;
        Team_N = (laps * l_banner_ft) / RAC;

        fprintf('============================================\n');
        fprintf('       SCORING MISIÓN 3: BANNER FLIGHT\n');
        fprintf('============================================\n');
        fprintf(' Banner:       %.1f m (%.1f ft)\n', l_banner, l_banner_ft);
        fprintf(' RAC:          %.2f lbs\n', RAC);
        fprintf(' Vueltas:      %d (en %.0f s)\n', laps, t_max);
        fprintf('--------------------------------------------\n');
        fprintf(' >> FACTOR N:    %.4f <<\n', Team_N);
        fprintf('============================================\n\n');

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

plot_simulation(inform, t_por_vuelta, vueltas_completadas, ...
                MTOW, rho, S_ref, CL_max, t);

