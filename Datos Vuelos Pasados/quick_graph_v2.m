%% quick_graph.m
% Herramienta rápida para graficar datos de vuelo.
% Definí los paneles que querés ver y listo.
%
% REQUISITO: leer_vuelo.m debe estar en el mismo directorio
%
% USO:
%   1. Ajustar csv_file
%   2. Definir panels: cada fila = un subplot, cada celda = una variable
%   3. (Opcional) Definir t_range para hacer zoom temporal
%   4. Correr
%
% NOMBRES DE COLUMNA DISPONIBLES:
%   t_s, Airspeed_kmh, Altitude_m, R_angle, P_angle,
%   Current_A, VFAS_V, VSpeed_ms, Throttle, Elevator,
%   Aileron, Rudder, AccX_g, AccY_g, AccZ_g,
%   LiPo1..LiPo8, LS1, LS2

clc; close all;

%% ========================================================================
%  CONFIGURACIÓN RÁPIDA — Tocar solo esto
%  ========================================================================

csv_file = 'CONDOR-S-2026-04-02-15-35-21.csv';

% --- QUÉ GRAFICAR ---
% Cada fila es un subplot. Cada celda dentro es una variable.
%
% Ejemplos:
%
%   Solo airspeed:
%     panels = { {'Airspeed_kmh'} };
%
%   Roll y pitch juntos, airspeed aparte:
%     panels = { {'R_angle', 'P_angle'}
%                {'Airspeed_kmh'} };
%
%   Zoom a la segunda vuelta:
%     panels = { {'Throttle'}
%                {'Airspeed_kmh'}
%                {'Current_A'} };
%     t_range = [74, 133];

panels = {
    {'Throttle'}
    {'Airspeed_kmh','Altitude_m'}
    {'R_angle', 'P_angle'}
    {'AccX_g', 'AccY_g', 'AccZ_g'}
};

% --- RANGO TEMPORAL (opcional) ---
% Dejar vacío [] para ver todo el vuelo
% Poner [t_inicio, t_fin] para hacer zoom
t_range = [];

% --- OFFSETS DE SENSORES (opcional) ---
% Dejar [] para cálculo automático (promedio primeros 5s en tierra)
% Poner un número para forzar un valor manual
roll_offset  = [];
pitch_offset = [];

% --- MARCAS DE VUELTA (opcional) ---
show_lap_marks = true;

%% ========================================================================
%  EJECUCIÓN — No tocar de acá para abajo
%  ========================================================================

% Leer datos
if ~exist('data', 'var')
    data = leer_vuelo(csv_file);
end
t = data.t_s;

% Calcular offsets automáticos si no fueron definidos manualmente
if isempty(roll_offset) && ismember('R_angle', data.Properties.VariableNames)
    n_ground = sum(t < 5);
    roll_offset = mean(data.R_angle(1:n_ground));
end
if isempty(pitch_offset) && ismember('P_angle', data.Properties.VariableNames)
    n_ground = sum(t < 5);
    pitch_offset = mean(data.P_angle(1:n_ground));
end

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
     'Airspeed_kmh', 'Altitude_m', ...
     'R_angle', 'P_angle', ...
     'Current_A', 'VFAS_V', ...
     'AccX_g', 'AccY_g', 'AccZ_g', ...
     'VSpeed_ms'}, ...
    {'Throttle', 'Elevator', 'Aileron', 'Rudder', ...
     'Airspeed [km/h]', 'Altitud [m]', ...
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
