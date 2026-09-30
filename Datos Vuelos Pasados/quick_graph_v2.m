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
    {'Throttle_2048'}
    {{'Airspeed_kmh'},{'Altitude_m'}}
    {'R_angle', 'P_angle'}
    {'AccX_g', 'AccY_g', 'AccZ_g'}
    {'Current_A'}
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

% --- DISTANCIA HASTA V_marca o z_marca (opcional - vacio no hace nada) ---
V_marca_kmh = [60];   % En km/h
z_marca_m = [30];   % En m

%% ========================================================================
%  EJECUCIÓN — No tocar de acá para abajo
%  ========================================================================

% Leer datos
if ~exist('data', 'var')
    data = leer_vuelo(csv_file);
end
t = data.t_s;

% --- Distancia horizontal recorrida hasta V_marca ---
idx_marca = [];
if (~isempty(V_marca_kmh) || ~isempty(z_marca_m)) ...
        && ismember('Airspeed_kmh', data.Properties.VariableNames) ...
        && ismember('VSpeed_ms', data.Properties.VariableNames)
    V_total = data.Airspeed_kmh / 3.6;
    VSpeed  = data.VSpeed_ms;
    V_horiz = sqrt(max(V_total.^2 - VSpeed.^2, 0));

    dt_vec = diff(t);
    dist_acum = [0; cumsum(0.5*(V_horiz(1:end-1)+V_horiz(2:end)).*dt_vec)];

    idx_marca = find(data.Airspeed_kmh >= V_marca_kmh, 1, 'first');
    if isempty(idx_marca)
        fprintf('V_marca = %.1f km/h no se alcanza en este vuelo.\n', V_marca_kmh);
    else
        fprintf('Distancia horizontal hasta alcanzar V_marca = %.1f km/h: %.1f m (en t = %.2f s)\n', ...
                V_marca_kmh, dist_acum(idx_marca), t(idx_marca));
    end
    idx_marca_z = [];
    if ~isempty(z_marca_m) && ismember('Altitude_m', data.Properties.VariableNames)
        idx_marca_z = find(data.Altitude_m >= z_marca_m, 1, 'first');
        if isempty(idx_marca_z)
            fprintf('z_marca = %.1f m no se alcanza en este vuelo.\n', z_marca_m);
        else
            fprintf('Distancia horizontal hasta alcanzar z_marca = %.1f m: %.1f m (en t = %.2f s)\n', ...
                    z_marca_m, dist_acum(idx_marca_z), t(idx_marca_z));
        end
    end
end

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
    {'Throttle', 'Throttle_2048', 'Elevator', 'Aileron', 'Rudder', ...
     'Airspeed_kmh', 'Altitude_m', ...
     'R_angle', 'P_angle', ...
     'Current_A', 'VFAS_V', ...
     'AccX_g', 'AccY_g', 'AccZ_g', ...
     'VSpeed_ms'}, ...
    {'Throttle', 'Throttle [0-2048]', 'Elevator', 'Aileron', 'Rudder', ...
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

    panel_p = panels{p};

    % Panel de doble eje: dos grupos de variables dentro de una celda,
    % ej. {{'Airspeed_kmh'}, {'Altitude_m'}}
    is_dual = iscell(panel_p) && numel(panel_p) == 2 && ...
              iscell(panel_p{1}) && iscell(panel_p{2});

    if is_dual
        yyaxis left
        legend_left = plot_vars_panel(panel_p{1}, data, mask, t_plot, ...
            roll_offset, pitch_offset, label_map, colors, 1);
        ylabel(strjoin(legend_left, ' / '));

        yyaxis right
        legend_right = plot_vars_panel(panel_p{2}, data, mask, t_plot, ...
            roll_offset, pitch_offset, label_map, colors, length(panel_p{1}) + 1);
        ylabel(strjoin(legend_right, ' / '));

        legend_entries = [legend_left, legend_right];
        if length(legend_entries) > 1
            legend(legend_entries, 'Location', 'eastoutside', 'FontSize', 7);
        end
    else
        vars = panel_p;
        legend_entries = plot_vars_panel(vars, data, mask, t_plot, ...
            roll_offset, pitch_offset, label_map, colors, 1);

        if length(vars) > 1
            legend(legend_entries, 'Location', 'eastoutside', 'FontSize', 7);
        end

        if length(vars) == 1 && label_map.isKey(vars{1})
            ylabel(label_map(vars{1}));
        else
            ylabel(strjoin(legend_entries, ' / '));
        end
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

% --- Marca visual de V_marca en los paneles que tengan Airspeed ---
if ~isempty(idx_marca)
    for p = 1:n_panels
        panel_p = panels{p};
        vars_check = panel_p;
        if iscell(panel_p) && numel(panel_p)==2 && iscell(panel_p{1}) && iscell(panel_p{2})
            vars_check = [panel_p{1}, panel_p{2}];
        end
        if any(strcmp(vars_check, 'Airspeed_kmh'))
            xline(ax(p), t(idx_marca), '--', sprintf('%.0fm', dist_acum(idx_marca)), ...
                  'Color', [0.2 0.6 0.2], 'LineWidth', 1.0, ...
                  'LabelOrientation', 'horizontal', ...
                  'LabelVerticalAlignment', 'top', 'FontSize', 7);
        end
    end
end

if ~isempty(idx_marca_z)
    for p = 1:n_panels
        panel_p = panels{p};
        vars_check = panel_p;
        if iscell(panel_p) && numel(panel_p)==2 && iscell(panel_p{1}) && iscell(panel_p{2})
            vars_check = [panel_p{1}, panel_p{2}];
        end
        if any(strcmp(vars_check, 'Altitude_m'))
            xline(ax(p), t(idx_marca_z), '--', sprintf('%.0fm', dist_acum(idx_marca_z)), ...
                  'Color', [0.85 0.33 0.10], 'LineWidth', 1.0, ...
                  'LabelOrientation', 'horizontal', 'LabelVerticalAlignment', 'top', 'FontSize', 7);
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


function legend_entries = plot_vars_panel(vars, data, mask, t_plot, roll_offset, pitch_offset, label_map, colors, color_start)
    legend_entries = {};
    for v = 1:length(vars)
        var_name = vars{v};

        if ~ismember(var_name, data.Properties.VariableNames)
            warning('Columna "%s" no encontrada. Columnas disponibles:', var_name);
            disp(data.Properties.VariableNames');
            continue;
        end

        y = data.(var_name);
        y_plot = y(mask);

        if ~isempty(roll_offset) && contains(var_name, 'R_angle')
            y_plot = y_plot - roll_offset;
            y_plot = mod(y_plot + 180, 360) - 180;
        end
        if ~isempty(pitch_offset) && contains(var_name, 'P_angle')
            y_plot = y_plot - pitch_offset;
        end

        plot(t_plot, y_plot, 'Color', colors(color_start + v - 1, :), 'LineWidth', 1.0);

        if label_map.isKey(var_name)
            legend_entries{end+1} = label_map(var_name);
        else
            legend_entries{end+1} = strrep(var_name, '_', ' ');
        end
    end
end