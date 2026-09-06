function plot_simulation(inform, t_por_vuelta, vueltas_completadas, ...
                         MTOW, rho, S_ref, CL_max, t_total)
% PLOT_SIMULATION Genera gráficos de la simulación de vuelo.
%
% USO:
%   plot_simulation(inform, t_por_vuelta, vueltas_completadas, ...
%                   MTOW, rho, S_ref, CL_max, t_total)
%
% ENTRADAS:
%   inform              - Matriz de log (27 filas x N columnas)
%   t_por_vuelta        - Vector con tiempo de cada vuelta [s]
%   vueltas_completadas - Cantidad de vueltas completadas
%   MTOW                - Masa total [kg]
%   rho                 - Densidad del aire [kg/m³]
%   S_ref               - Superficie alar [m²]
%   CL_max              - CL máximo de la polar (para línea de referencia)
%   t_total             - Tiempo total de simulación [s]
%
% GENERA:
%   Figura 1: Trayectoria x-y
%   Figura 2: 6 subplots sincronizados (V, CL/CD, L/D, T/D, I/E, Roll)

    if isempty(inform)
        warning('plot_simulation: inform está vacío, no se generan gráficos.');
        return;
    end

    g = 9.81;

    % =====================================================================
    %  EXTRAER VARIABLES DEL LOG
    % =====================================================================
    %  Filas de inform (definidas en airplane_dynamics_opt):
    %   1-3:   x, y, z
    %   4-6:   v_x, v_y, v_z
    %   7-9:   a_x, a_y, a_z
    %   10:    t
    %   11:    cl
    %   12:    cd_total
    %   13:    drag [N]
    %   14:    thrust [N]
    %   15-17: LIFT_vec (x,y,z)
    %   18-20: THRUST_vec (x,y,z)
    %   21-23: DRAG_vec (x,y,z)
    %   24:    corriente [A]
    %   25-26: viento (x,y)
    %   27:    roll_rad

    log_x       = inform(1, :);
    log_y       = inform(2, :);
    log_vx      = inform(4, :);
    log_vy      = inform(5, :);
    log_vz      = inform(6, :);
    log_t       = inform(10, :);
    log_cl      = inform(11, :);
    log_cd      = inform(12, :);
    log_drag    = inform(13, :);
    log_thrust  = inform(14, :);
    log_current = inform(24, :);
    log_roll    = inform(27, :);

    % Variables derivadas
    log_v_mag   = sqrt(log_vx.^2 + log_vy.^2 + log_vz.^2);
    log_LD      = log_cl ./ max(log_cd, 1e-6);   % L/D, con protección contra cd=0
    log_roll_deg = rad2deg(log_roll);

    % Energía acumulada
    log_dt     = [0, diff(log_t)];
    log_energy = cumsum(log_current .* log_dt) / 3600;

    % Marcas de inicio de cada vuelta
    t_marcas = zeros(1, vueltas_completadas);
    t_acum = 0;
    for k = 1:vueltas_completadas
        t_acum = t_acum + t_por_vuelta(k);
        t_marcas(k) = t_acum;
    end

    % =====================================================================
    %  FIGURA 1: TRAYECTORIA X-Y
    % =====================================================================
    figure('Name', 'Trayectoria', 'NumberTitle', 'off', ...
           'Units', 'normalized', 'Position', [0.02 0.25 0.4 0.55]);

    plot(log_y, log_x, 'b-', 'LineWidth', 1.5);
    hold on; grid on; axis equal;

    % Inicio y fin
    plot(log_y(1), log_x(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(log_y(end), log_x(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

    % Flecha de dirección inicial
    quiver(log_y(1), log_x(1), log_vy(1)*2, log_vx(1)*2, ...
           'g', 'LineWidth', 2, 'MaxHeadSize', 2);

    xlabel('y [m] (lateral)');
    ylabel('x [m] (avance)');
    title(sprintf('Trayectoria — %d vueltas, %.1f s', vueltas_completadas, t_total));
    legend('Trayectoria', 'Inicio', 'Fin', 'Location', 'best');
    hold off;

    % =====================================================================
    %  FIGURA 2: VARIABLES VS TIEMPO (6 subplots)
    % =====================================================================
    figure('Name', 'Performance de Vuelo', 'NumberTitle', 'off', ...
           'Units', 'normalized', 'Position', [0.45 0.03 0.5 0.92]);

    n_panels = 6;
    ax = gobjects(n_panels, 1);

    % --- Panel 1: Velocidad ---
    ax(1) = subplot(n_panels, 1, 1);
    plot(log_t, log_v_mag, 'Color', [0.2 0.4 0.8], 'LineWidth', 1.2);
    grid on;
    ylabel('V [m/s]');
    title('Velocidad');

        % --- Panel 2: CL y CD ---
    ax(2) = subplot(n_panels, 1, 2);
    yyaxis left
    plot(log_t, log_cl, 'Color', [0.8 0.2 0.2], 'LineWidth', 1.0);
    hold on;
    yline(CL_max, '--r', sprintf('CL_{max} = %.2f', CL_max), ...
          'LineWidth', 1.0, 'LabelHorizontalAlignment', 'left', 'FontSize', 7);
    ylabel('C_L');
    ax(2).YAxis(1).Color = [0.8 0.2 0.2];

    yyaxis right
    plot(log_t, log_cd, 'Color', [0.2 0.6 0.3], 'LineWidth', 1.0);
    ylabel('C_D');
    ax(2).YAxis(2).Color = [0.2 0.6 0.3];
    grid on;
    title('Coeficientes Aerodinámicos');

    % --- Panel 3: Eficiencia aerodinámica L/D ---
    ax(3) = subplot(n_panels, 1, 3);
    plot(log_t, log_LD, 'Color', [0.0 0.15 0.45], 'LineWidth', 1.2);
    grid on;
    ylabel('L/D');
    title('Eficiencia Aerodinámica');

    % --- Panel 4: Thrust y Drag ---
    ax(4) = subplot(n_panels, 1, 4);
    plot(log_t, log_thrust, 'Color', [0.1 0.5 0.1], 'LineWidth', 1.0);
    hold on;
    plot(log_t, log_drag, 'Color', [0.8 0.2 0.1], 'LineWidth', 1.0);
    grid on;
    ylabel('Fuerza [N]');
    legend('Thrust', 'Drag', 'Location', 'eastoutside', 'FontSize', 7);
    title('Fuerzas Propulsivas y Resistencia');

        % --- Panel 5: Corriente y Energía ---
    ax(5) = subplot(n_panels, 1, 5);
    yyaxis left
    plot(log_t, log_current, 'Color', [0.8 0.3 0.1], 'LineWidth', 1.0);
    ylabel('Corriente [A]');
    ax(5).YAxis(1).Color = [0.8 0.3 0.1];

    yyaxis right
    plot(log_t, log_energy, 'Color', [0.5 0.1 0.5], 'LineWidth', 1.2);
    ylabel('Energía [Ah]');
    ax(5).YAxis(2).Color = [0.5 0.1 0.5];
    grid on;
    title('Consumo Eléctrico');

    % --- Panel 6: Roll ---
    ax(6) = subplot(n_panels, 1, 6);
    plot(log_t, log_roll_deg, 'Color', [0.6 0.2 0.6], 'LineWidth', 1.0);
    grid on;
    ylabel('Roll [°]');
    xlabel('Tiempo [s]');
    title('Bank Angle');

    % --- Marcas de vuelta en todos los paneles ---
    for p = 1:n_panels
        for k = 1:length(t_marcas)
            xline(ax(p), t_marcas(k), '--', 'Color', [0.5 0.5 0.5], ...
                  'LineWidth', 0.7, 'Alpha', 0.6);
        end
    end

    % Etiquetas solo en panel 1
    for k = 1:length(t_marcas)
        xline(ax(1), t_marcas(k), '--', sprintf('V%d', k), ...
              'Color', [0.5 0.5 0.5], 'LineWidth', 0.7, ...
              'LabelOrientation', 'horizontal', ...
              'LabelVerticalAlignment', 'bottom', 'FontSize', 7);
    end

    % Sincronizar zoom
    linkaxes(ax, 'x');

end