%% quick_graph.m
% Herramienta rápida para graficar datos de vuelo.
% Definí los paneles que querés ver y listo.
%
% USO:
%   1. Ajustar csv_file
%   2. Definir panels: cada fila = un subplot, cada celda = una variable
%   3. (Opcional) Definir t_range para hacer zoom temporal
%   4. (Opcional) Definir offsets de sensores
%   5. Correr
%
% Para ver los nombres de columna disponibles, correr:
%   data = readtable('vuelo_procesado.csv');
%   disp(data.Properties.VariableNames');

clc; close all;

%% ========================================================================
%  CONFIGURACIÓN RÁPIDA — Tocar solo esto
%  ========================================================================

csv_file = 'vuelo_procesado.csv';

% --- QUÉ GRAFICAR ---
% Cada fila es un subplot. Cada celda dentro es una variable.
% Usar los nombres de columna del CSV tal cual aparecen en MATLAB.
%
% Ejemplos:
%
%   Solo airspeed:
%     panels = { {'Airspeed_km_h_'} };
%
%   Roll y pitch juntos, airspeed aparte:
%     panels = { {'R_angle___', 'P_angle___'}
%                {'Airspeed_km_h_'} };
%
%   Todo lo que quieras:
%     panels = { {'Throttle'}
%                {'Airspeed_km_h_'}
%                {'Altitude_m_'}
%                {'R_angle___', 'P_angle___'}
%                {'Current_A_', 'VFAS_V_'} };

panels = {
    {'Throttle'}
    {'Airspeed_km_h_'}
    {'Altitude_m_'}
    {'R_angle___', 'P_angle___'}
    {'Current_A_'}
};

% --- RANGO TEMPORAL (opcional) ---
% Dejar vacío [] para ver todo el vuelo
% Poner [t_inicio, t_fin] para hacer zoom
t_range = [];           % Ejemplos: [70, 140] o [0, 50] o []

% --- OFFSETS DE SENSORES (opcional) ---
% Si están definidos, se aplican automáticamente a R.angle y P.angle
% Poner [] para NO corregir
roll_offset  = -166;    % [°] o []
pitch_offset = 10;      % [°] o []

% --- MARCAS DE VUELTA (opcional) ---
% Si es true, dibuja líneas verticales en las transiciones de LS1
show_lap_marks = true;

%% ========================================================================
%  EJECUCIÓN — No tocar de acá para abajo
%  ========================================================================

% Leer datos
if ~exist('data', 'var')
    data = readtable(csv_file);
    fprintf('Archivo cargado: %s (%d filas)\n', csv_file, height(data));
end

% Buscar columna de tiempo
t_col_candidates = {'t_s_', 'ts', 't'};
t_col = '';
for k = 1:length(t_col_candidates)
    if ismember(t_col_candidates{k}, data.Properties.VariableNames)
        t_col = t_col_candidates{k};
        break;
    end
end
if isempty(t_col)
    % Si no encuentra, usar la primera columna numérica
    t_col = data.Properties.VariableNames{1};
    fprintf('⚠ No encontré columna t(s), usando "%s"\n', t_col);
end
t = data.(t_col);

% Aplicar rango temporal
if ~isempty(t_range)
    mask = t >= t_range(1) & t <= t_range(2);
else
    mask = true(size(t));
end
t_plot = t(mask);

% Mapa de nombres bonitos para los labels
label_map = containers.Map( ...
    {'Throttle', 'Elevator', 'Aileron', 'Rudder', ...
     'Airspeed_km_h_', 'Air_speed_km_h_', 'AirSpeed_km_h_', ...
     'Altitude_m_', ...
     'R_angle___', 'P_angle___', ...
     'Current_A_', 'VFAS_V_', ...
     'AccX_g_', 'AccY_g_', 'AccZ_g_', ...
     'VSpeed_m_s_'}, ...
    {'Throttle', 'Elevator', 'Aileron', 'Rudder', ...
     'Airspeed [km/h]', 'Airspeed [km/h]', 'Airspeed [km/h]', ...
     'Altitud [m]', ...
     'Roll [°]', 'Pitch [°]', ...
     'Corriente [A]', 'Voltaje [V]', ...
     'AccX [g]', 'AccY [g]', 'AccZ [g]', ...
     'VSpeed [m/s]'} ...
);

% Detectar marcas de vuelta
t_laps = [];
if show_lap_marks && ismember('LS1', data.Properties.VariableNames)
    ls1 = data.LS1;
    idx_ls1 = find(diff(ls1) > 0);
    if ~isempty(idx_ls1)
        t_laps = t(idx_ls1 + 1);
    end
end

% Crear figura
n_panels = length(panels);
fig = figure('Name', 'Quick Graph', 'NumberTitle', 'off', ...
             'Units', 'normalized', 'Position', [0.1 0.05 0.8 0.88]);

ax = gobjects(n_panels, 1);
colors = lines(8);

for p = 1:n_panels
    ax(p) = subplot(n_panels, 1, p);
    hold on; grid on;
    
    vars = panels{p};
    legend_entries = {};
    
    for v = 1:length(vars)
        var_name = vars{v};
        
        % Verificar que la columna existe
        if ~ismember(var_name, data.Properties.VariableNames)
            warning('Columna "%s" no encontrada. Columnas disponibles:', var_name);
            disp(data.Properties.VariableNames');
            continue;
        end
        
        % Leer datos
        y = data.(var_name);
        y_plot = y(mask);
        
        % Aplicar correcciones de offset si corresponde
        if ~isempty(roll_offset) && contains(var_name, 'R_angle')
            y_plot = y_plot - roll_offset;
            y_plot = mod(y_plot + 180, 360) - 180;
        end
        if ~isempty(pitch_offset) && contains(var_name, 'P_angle')
            y_plot = y_plot - pitch_offset;
        end
        
        % Graficar
        plot(t_plot, y_plot, 'Color', colors(v,:), 'LineWidth', 1.0);
        
        % Label bonito
        if label_map.isKey(var_name)
            legend_entries{end+1} = label_map(var_name);
        else
            legend_entries{end+1} = strrep(var_name, '_', ' ');
        end
    end
    
    % Leyenda (solo si hay más de una variable en el panel)
    if length(vars) > 1
        legend(legend_entries, 'Location', 'eastoutside', 'FontSize', 7);
    end
    
    % Y-label: usar el label bonito si es una sola variable
    if length(vars) == 1 && label_map.isKey(vars{1})
        ylabel(label_map(vars{1}));
    else
        ylabel(strjoin(legend_entries, ' / '));
    end
    
    % Marcas de vuelta
    for k = 1:length(t_laps)
        if isempty(t_range) || (t_laps(k) >= t_range(1) && t_laps(k) <= t_range(2))
            xline(t_laps(k), '--', 'Color', [0.6 0.6 0.6], ...
                  'LineWidth', 0.7, 'Alpha', 0.6);
            if p == 1
                xline(t_laps(k), '--', sprintf('V%d', k), ...
                      'Color', [0.6 0.6 0.6], 'LineWidth', 0.7, ...
                      'LabelOrientation', 'horizontal', ...
                      'LabelVerticalAlignment', 'bottom', 'FontSize', 7);
            end
        end
    end
end

% X-label solo en el último panel
xlabel(ax(end), 'Tiempo [s]');

% Sincronizar zoom
linkaxes(ax, 'x');

% Ajustar rango x
if ~isempty(t_range)
    xlim(ax(1), t_range);
end

fprintf('Graficando %d paneles', n_panels);
if ~isempty(t_range)
    fprintf(' | t = [%.1f, %.1f] s', t_range(1), t_range(2));
end
fprintf(' | %d marcas de vuelta\n', length(t_laps));
