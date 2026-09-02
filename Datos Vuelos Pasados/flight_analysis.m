%% flight_analysis.m
% Análisis completo de datos de vuelo — Aero ITBA DBF
% Lee el CSV procesado (con t(s) como primera columna) y genera
% 6 subplots sincronizados con las variables clave del vuelo.
%
% USO:
%   1. Ajustar la sección CONFIGURACIÓN (archivo, offsets)
%   2. Correr el script
%   3. Usar zoom en cualquier panel — todos se sincronizan

clc; clear; close all;

%% ========================================================================
%  CONFIGURACIÓN — Modificar según el vuelo
%  ========================================================================

csv_file = 'vuelo_procesado.csv';   % Archivo CSV con t(s) como col 1

% Offsets de sensores (medir de los primeros segundos en tierra)
roll_offset  = -166;    % [°] R.angle en tierra (restar para corregir)
pitch_offset = 10;      % [°] P.angle en tierra (restar para corregir)

% Rango de throttle del transmisor (para normalizar a 0-100%)
throttle_min = -1024;   % Valor de throttle con stick abajo
throttle_max = 1024;    % Valor de throttle con stick arriba

% Umbrales para detección automática de fases
throttle_idle_threshold = -500;   % Por debajo de esto = motor apagado
current_cutoff_threshold = 5;     % [A] Por debajo = motor cortado

%% ========================================================================
%  LECTURA DE DATOS
%  ========================================================================

data = readtable(csv_file);

t           = data.t_s_;
airspeed_kph = data.Airspeed_km_h_;
altitude    = data.Altitude_m_;
roll_raw    = data.R_angle___;
pitch_raw   = data.P_angle___;
current     = data.Current_A_;
vfas        = data.VFAS_V_;
throttle_raw = data.Throttle;
elevator_raw = data.Elevator;
aileron_raw  = data.Aileron;
rudder_raw   = data.Rudder;
ls1         = data.LS1;
ls2         = data.LS2;

%% ========================================================================
%  PROCESAMIENTO
%  ========================================================================

% Airspeed a m/s
airspeed_ms = airspeed_kph / 3.6;

% Corregir roll y pitch por offset del sensor
roll_corr = roll_raw - roll_offset;
% Normalizar a [-180, 180]
roll_corr = mod(roll_corr + 180, 360) - 180;

pitch_corr = pitch_raw - pitch_offset;

% Normalizar controles del piloto a porcentaje
throttle_pct = (throttle_raw - throttle_min) / (throttle_max - throttle_min) * 100;
elevator_pct = elevator_raw / throttle_max * 100;
aileron_pct  = aileron_raw / throttle_max * 100;
rudder_pct   = rudder_raw / throttle_max * 100;

% Energía acumulada (integral de corriente)
dt = [0; diff(t)];
energy_Ah = cumsum(current .* dt) / 3600;

% Detectar eventos clave
% Throttle sube de idle
idx_throttle_up = find(throttle_raw(1:end-1) < throttle_idle_threshold & ...
                       throttle_raw(2:end) >= throttle_idle_threshold, 1, 'first');
if ~isempty(idx_throttle_up)
    t_throttle_up = t(idx_throttle_up + 1);
else
    t_throttle_up = [];
end

% LS2 se activa (inicio de vuelo)
idx_ls2 = find(diff(ls2) > 0, 1, 'first');
if ~isempty(idx_ls2)
    t_ls2 = t(idx_ls2 + 1);
else
    t_ls2 = [];
end

% LS1 se activa (marcas de vuelta)
idx_ls1 = find(diff(ls1) > 0);
t_ls1 = t(idx_ls1 + 1);

% Motor cortado (corriente baja sostenida)
idx_cutoff = find(current(1:end-1) > current_cutoff_threshold & ...
                  current(2:end) <= current_cutoff_threshold);
% Filtrar: buscar el último corte significativo (no un glitch)
t_cutoff = [];
for k = 1:length(idx_cutoff)
    idx = idx_cutoff(k);
    if idx + 10 <= length(current)
        if mean(current(idx:idx+10)) < current_cutoff_threshold
            t_cutoff = t(idx);
        end
    end
end

%% ========================================================================
%  GRÁFICOS
%  ========================================================================

fig = figure('Name', 'Análisis de Vuelo', 'NumberTitle', 'off', ...
             'Units', 'normalized', 'Position', [0.05 0.05 0.9 0.85]);

ax = gobjects(6,1);
colors = lines(4);

% --- Panel 1: Controles del piloto ---
ax(1) = subplot(6,1,1);
hold on; grid on;
plot(t, throttle_pct, 'Color', colors(1,:), 'LineWidth', 1.2);
plot(t, elevator_pct, 'Color', colors(2,:), 'LineWidth', 0.8);
plot(t, aileron_pct,  'Color', colors(3,:), 'LineWidth', 0.8);
plot(t, rudder_pct,   'Color', colors(4,:), 'LineWidth', 0.8);
ylabel('Control [%]');
legend('Throttle', 'Elevator', 'Aileron', 'Rudder', ...
       'Location', 'eastoutside', 'FontSize', 7);
title('Controles del Piloto');

% --- Panel 2: Airspeed ---
ax(2) = subplot(6,1,2);
plot(t, airspeed_ms, 'Color', [0.2 0.4 0.8], 'LineWidth', 1.2);
grid on;
ylabel('Airspeed [m/s]');
title('Velocidad del Aire (Pitot)');

% --- Panel 3: Altitud ---
ax(3) = subplot(6,1,3);
plot(t, altitude, 'Color', [0.2 0.6 0.3], 'LineWidth', 1.2);
grid on;
ylabel('Altitud [m]');
title('Altitud Barométrica (relativa)');

% --- Panel 4: Actitud ---
ax(4) = subplot(6,1,4);
hold on; grid on;
plot(t, roll_corr,  'Color', [0.8 0.2 0.2], 'LineWidth', 1.2);
plot(t, pitch_corr, 'Color', [0.9 0.6 0.1], 'LineWidth', 1.0);
ylabel('Ángulo [°]');
legend('Roll (corr.)', 'Pitch (corr.)', 'Location', 'eastoutside', 'FontSize', 7);
title('Actitud del Avión (corregida por offset)');

% --- Panel 5: Propulsión ---
ax(5) = subplot(6,1,5);
yyaxis left
plot(t, current, 'Color', [0.8 0.3 0.1], 'LineWidth', 1.0);
ylabel('Corriente [A]');
yyaxis right
plot(t, vfas, 'Color', [0.1 0.3 0.8], 'LineWidth', 1.0);
ylabel('Voltaje [V]');
grid on;
title('Propulsión: Corriente y Voltaje del Pack');

% --- Panel 6: Energía acumulada ---
ax(6) = subplot(6,1,6);
plot(t, energy_Ah, 'Color', [0.5 0.1 0.5], 'LineWidth', 1.2);
grid on;
ylabel('Energía [Ah]');
xlabel('Tiempo [s]');
title('Energía Consumida Acumulada');

% --- Líneas verticales de referencia en TODOS los paneles ---
event_times = [];
event_labels = {};

if ~isempty(t_throttle_up)
    event_times(end+1) = t_throttle_up;
    event_labels{end+1} = 'Throttle UP';
end
if ~isempty(t_ls2)
    event_times(end+1) = t_ls2;
    event_labels{end+1} = 'LS2 (inicio)';
end
for k = 1:length(t_ls1)
    event_times(end+1) = t_ls1(k);
    event_labels{end+1} = sprintf('LS1 vuelta %d', k);
end
if ~isempty(t_cutoff)
    event_times(end+1) = t_cutoff;
    event_labels{end+1} = 'Motor OFF';
end

for p = 1:6
    for e = 1:length(event_times)
        xline(ax(p), event_times(e), '--', 'Color', [0.5 0.5 0.5], ...
              'LineWidth', 0.8, 'Alpha', 0.7);
    end
end

% Etiquetas de eventos solo en el panel superior
for e = 1:length(event_times)
    xline(ax(1), event_times(e), '--', event_labels{e}, ...
          'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, ...
          'LabelOrientation', 'horizontal', ...
          'LabelVerticalAlignment', 'bottom', 'FontSize', 7);
end

% Sincronizar zoom en todos los paneles
linkaxes(ax, 'x');

% Resumen en consola
fprintf('\n=== RESUMEN DEL VUELO ===\n');
fprintf('Duración total: %.1f s (%.1f min)\n', t(end), t(end)/60);
fprintf('Energía consumida: %.3f Ah\n', energy_Ah(end));
if ~isempty(t_throttle_up)
    fprintf('Throttle UP en t = %.1f s\n', t_throttle_up);
end
if ~isempty(t_ls2)
    fprintf('LS2 (inicio vuelo) en t = %.1f s\n', t_ls2);
end
for k = 1:length(t_ls1)
    fprintf('LS1 (vuelta %d) en t = %.1f s\n', k, t_ls1(k));
end
if length(t_ls1) >= 2
    fprintf('Tiempo de vuelta (entre marcas): %.1f s\n', t_ls1(2) - t_ls1(1));
end
if ~isempty(t_cutoff)
    fprintf('Motor OFF en t = %.1f s\n', t_cutoff);
end
fprintf('Corriente media (en vuelo): %.1f A\n', mean(current(t > 32 & current > 5)));
fprintf('Airspeed media (en vuelo): %.1f m/s (%.1f km/h)\n', ...
    mean(airspeed_ms(t > 32 & airspeed_ms > 3)), ...
    mean(airspeed_ms(t > 32 & airspeed_ms > 3)) * 3.6);
fprintf('Voltaje min bajo carga: %.2f V (%.3f V/cel)\n', min(vfas(current > 10)), min(vfas(current > 10))/8);
fprintf('========================\n');
