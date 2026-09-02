function data = leer_vuelo(csv_file)
% LEER_VUELO Lee un CSV de vuelo (crudo o procesado) y devuelve una tabla
% normalizada con nombres de columna estándar y t(s) como columna.
%
% USO:
%   data = leer_vuelo('CONDOR-S-2026-04-02-15-35-21.csv');
%   data = leer_vuelo('vuelo_procesado.csv');
%
% SALIDA:
%   Tabla con columnas estandarizadas:
%     t_s, Airspeed_kmh, Altitude_m, R_angle, P_angle,
%     Current_A, VFAS_V, VSpeed_ms, Throttle, Elevator,
%     Aileron, Rudder, AccX_g, AccY_g, AccZ_g,
%     LiPo1..LiPo8, LS1, LS2, y todas las demás que tenga el CSV.

    % Leer tabla cruda
    raw = readtable(csv_file, 'VariableNamingRule', 'preserve');
    fprintf('Leyendo: %s (%d filas, %d columnas)\n', csv_file, height(raw), width(raw));

    % Mapa de nombres: {posibles nombres crudos} → nombre estándar
    % Cada fila: nombre estándar, seguido de variantes que MATLAB puede generar
    rename_map = {
        't_s',          {'t(s)', 't_s_', 'ts'}
        'Airspeed_kmh', {'Air speed(km/h)', 'Airspeed_km_h_', 'Air_speed_km_h_', 'AirSpeed_km_h_'}
        'Altitude_m',   {'Altitude(m)', 'Altitude_m_'}
        'R_angle',      {'R.angle(°)', 'R_angle___', 'R_angle____'}
        'P_angle',      {'P.angle(°)', 'P_angle___', 'P_angle____'}
        'Current_A',    {'Current(A)', 'Current_A_'}
        'VFAS_V',       {'VFAS(V)', 'VFAS_V_'}
        'VSpeed_ms',    {'VSpeed(m/s)', 'VSpeed_m_s_'}
        'AccX_g',       {'AccX(g)', 'AccX_g_'}
        'AccY_g',       {'AccY(g)', 'AccY_g_'}
        'AccZ_g',       {'AccZ(g)', 'AccZ_g_'}
        'LiPo1',        {'LiPo1(V)', 'LiPo1_V_'}
        'LiPo2',        {'LiPo2(V)', 'LiPo2_V_'}
        'LiPo3',        {'LiPo3(V)', 'LiPo3_V_'}
        'LiPo4',        {'LiPo4(V)', 'LiPo4_V_'}
        'LiPo5',        {'LiPo5(V)', 'LiPo5_V_'}
        'LiPo6',        {'LiPo6(V)', 'LiPo6_V_'}
        'LiPo7',        {'LiPo7(V)', 'LiPo7_V_'}
        'LiPo8',        {'LiPo8(V)', 'LiPo8_V_'}
        'TxBat_V',      {'TxBat(V)', 'TxBat_V_'}
        'RxBatt_V',     {'RxBatt(V)', 'RxBatt_V_'}
        'Temp1',        {'Temp1(°C)', 'Temp1___C_', 'Temp1__C_'}
        'Temp2',        {'Temp2(°C)', 'Temp2___C_', 'Temp2__C_'}
    };
    % Columnas que ya tienen nombres simples y no necesitan mapeo:
    % Throttle, Elevator, Aileron, Rudder, LS1, LS2, SA-SJ, etc.

    % Aplicar renombre
    col_names = raw.Properties.VariableNames;
    for k = 1:size(rename_map, 1)
        std_name = rename_map{k, 1};
        variants = rename_map{k, 2};
        for v = 1:length(variants)
            idx = strcmp(col_names, variants{v});
            if any(idx)
                raw.Properties.VariableNames{idx} = std_name;
                col_names = raw.Properties.VariableNames;  % refrescar
                break;
            end
        end
    end

    % Generar columna t_s si no existe
    if ~ismember('t_s', raw.Properties.VariableNames)
        if ismember('Time', raw.Properties.VariableNames)
            time_raw = raw.Time;
            % Manejar si es string, char, o cell
            if iscell(time_raw)
                time_strings = strtrim(time_raw);
            elseif isstring(time_raw)
                time_strings = cellstr(strtrim(time_raw));
            else
                time_strings = cellstr(time_raw);
            end
            
            % Parsear timestamps
            t_datetime = datetime(time_strings, 'InputFormat', 'HH:mm:ss.SSS');
            t_ref = t_datetime(1);
            t_sec = seconds(t_datetime - t_ref);
            
            % Insertar como primera columna
            raw.t_s = t_sec;
            % Mover t_s al principio
            raw = movevars(raw, 't_s', 'Before', 1);
            fprintf('Columna t_s generada desde Time (0 a %.1f s)\n', t_sec(end));
        else
            error('No encontré columna Time ni t_s en el archivo.');
        end
    end

    % Eliminar columnas vacías (la última columna a veces es vacía por la coma final)
    col_names = raw.Properties.VariableNames;
    for k = width(raw):-1:1
        name = col_names{k};
        if startsWith(name, 'Var') || isempty(strtrim(name))
            raw(:, k) = [];
        end
    end

    data = raw;

    % Resumen
    fprintf('Columnas disponibles:\n');
    cols = data.Properties.VariableNames;
    for k = 1:length(cols)
        fprintf('  %2d. %s\n', k, cols{k});
    end
    fprintf('\n');
end
