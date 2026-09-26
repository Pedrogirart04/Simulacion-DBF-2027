function plot_simulation(inform, t_por_vuelta, vueltas_completadas, ...
                         MTOW, rho, S_ref, CL_max, t_total, ...
                         plot1_on, plot2_on, panels)

% PLOT_SIMULATION Genera gráficos de la simulación de vuelo.
%
% USO:
%   plot_simulation(inform, t_por_vuelta, vueltas_completadas, ...
%                   MTOW, rho, S_ref, CL_max, t_total)
%
% ENTRADAS:
%   inform              - Matriz de log (29 filas x N columnas)
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

    log_x       = inform(1, :);    log_y       = inform(2, :);
    log_z       = inform(3, :);    log_vx      = inform(4, :);
    log_vy      = inform(5, :);    log_vz      = inform(6, :);
    log_t       = inform(10, :);    log_cl      = inform(11, :);
    log_cd      = inform(12, :);    log_drag    = inform(13, :);
    log_thrust  = inform(14, :);    log_current = inform(24, :);
    log_roll    = inform(28, :);

    % Variables derivadas
    log_v_mag   = sqrt(log_vx.^2 + log_vy.^2 + log_vz.^2);
    log_LD      = log_cl ./ max(log_cd, 1e-6);   % L/D, con protección contra cd=0
    log_roll_deg = rad2deg(log_roll);

    % Energía acumulada
    log_dt     = [0, diff(log_t)];
    log_energy = cumsum(log_current .* log_dt) / 3600;

        log_omega   = inform(29, :);
    log_windx   = inform(25, :);
    log_rpm     = log_omega * 60/(2*pi);

    % Mapa nombre -> vector, para el sistema de panels configurables
    logvars = struct( ...
        'V_ms',        log_v_mag, ...
        'CL',          log_cl, ...
        'CD',          log_cd, ...
        'LD',          log_LD, ...
        'Thrust_N',    log_thrust, ...
        'Drag_N',      log_drag, ...
        'Corriente_A', log_current, ...
        'Energia_Ah',  log_energy, ...
        'Roll_deg',    log_roll_deg, ...
        'Altitude_m',  log_z, ...
        'RPM',         log_rpm, ...
        'Viento_x_ms', log_windx);

    label_map = containers.Map( ...
        {'V_ms','CL','CD','LD','Thrust_N','Drag_N','Corriente_A', ...
         'Energia_Ah','Roll_deg','Altitude_m','RPM','Viento_x_ms'}, ...
        {'V [m/s]','C_L','C_D','L/D','Thrust [N]','Drag [N]','Corriente [A]', ...
         'Energía [Ah]','Roll [°]','Altitud [m]','RPM','Viento_x [m/s]'});


    % Marcas de inicio de cada vuelta
    t_marcas = zeros(1, vueltas_completadas);
    t_acum = 0;
    for k = 1:vueltas_completadas
        t_acum = t_acum + t_por_vuelta(k);
        t_marcas(k) = t_acum;
    end

    % =====================================================================
    %  FIGURA 1: TRAYECTORIAS 2D Y 3D
    % =====================================================================
    if plot1_on
        figure('Name', 'Trayectorias de Vuelo', 'NumberTitle', 'off', ...
               'Units', 'normalized', 'Position', [0.02 0.25 0.50 0.55]);
    
        % --- SUBPLOT 1: Vista Superior 2D (Planta) ---
        subplot(1, 2, 1);
        plot(log_y, log_x, 'b-', 'LineWidth', 1.5);
        hold on; grid on; axis equal;
    
        % Puntos Inicio y Fin
        plot(log_y(1), log_x(1), 'go', 'MarkerSize', 9, 'MarkerFaceColor', 'g');
        plot(log_y(end), log_x(end), 'rs', 'MarkerSize', 9, 'MarkerFaceColor', 'r');
    
        % Flecha de dirección inicial
        quiver(log_y(1), log_x(1), log_vy(1)*2, log_vx(1)*2, ...
               'g', 'LineWidth', 2, 'MaxHeadSize', 2);
    
        xlabel('y [m] (lateral)');
        ylabel('x [m] (avance)');
        title(sprintf('Vista 2D (Planta) — %d vueltas', vueltas_completadas));
        legend('Trayectoria', 'Inicio', 'Fin', 'Location', 'best');
        hold off;
    
        % --- SUBPLOT 2: Vista Espacial 3D ---
        subplot(1, 2, 2);
        plot3(log_y, log_x, log_z, 'b-', 'LineWidth', 1.5);
        hold on; grid on; axis equal;
    
        % Puntos Inicio y Fin 3D
        plot3(log_y(1), log_x(1), log_z(1), 'go', 'MarkerSize', 9, 'MarkerFaceColor', 'g');
        plot3(log_y(end), log_x(end), log_z(end), 'rs', 'MarkerSize', 9, 'MarkerFaceColor', 'r');
    
        % Sombra / Proyección en el suelo (Z = 0)
        plot3(log_y, log_x, zeros(size(log_z)), 'Color', [0.7 0.7 0.7], ...
              'LineStyle', '--', 'LineWidth', 0.8);
    
        xlabel('y [m] (lateral)');
        ylabel('x [m] (avance)');
        zlabel('z [m] (altitud)');
        title('Trayectoria 3D');
        legend('Trayectoria 3D', 'Inicio', 'Fin', 'Proyección Suelo', 'Location', 'best');
        
        view(45, 30); % Ángulo de cámara 3D (Azimuth 45°, Elevación 30°)
        hold off;
    end 

    % =====================================================================
    %  FIGURA 2: VARIABLES VS TIEMPO (6 subplots)
    % =====================================================================
    
    if plot2_on
        n_panels = length(panels);
        figure('Name', 'Performance de Vuelo', 'NumberTitle', 'off', ...
               'Units', 'normalized', 'Position', [0.45 0.03 0.5 0.92]);
    
        ax = gobjects(n_panels, 1);
        colors = lines(8);
    
            for p = 1:n_panels
        ax(p) = subplot(n_panels, 1, p);
        hold on; grid on;

        panel_p = panels{p};
        is_dual_axis = iscell(panel_p) && numel(panel_p) == 2 && ...
                       iscell(panel_p{1}) && iscell(panel_p{2});

        if is_dual_axis
            % --- Panel de doble eje: panel_p{1} = izquierda, panel_p{2} = derecha ---
            vars_left  = panel_p{1};
            vars_right = panel_p{2};

            yyaxis left
            legend_left = {};
            for v = 1:length(vars_left)
                var_name = vars_left{v};
                if ~isfield(logvars, var_name)
                    warning('plot_simulation: variable "%s" no reconocida, se omite.', var_name);
                    continue;
                end
                plot(log_t, logvars.(var_name), 'LineWidth', 1.0);
                if isKey(label_map, var_name)
                    legend_left{end+1} = label_map(var_name);
                else
                    legend_left{end+1} = strrep(var_name, '_', ' ');
                end
                if strcmp(var_name, 'CL')
                    yline(CL_max, '--r', sprintf('CL_{max} = %.2f', CL_max), ...
                          'LineWidth', 1.0, 'LabelHorizontalAlignment', 'left', 'FontSize', 7);
                end
            end
            ylabel(strjoin(legend_left, ' / '));

            yyaxis right
            legend_right = {};
            for v = 1:length(vars_right)
                var_name = vars_right{v};
                if ~isfield(logvars, var_name)
                    warning('plot_simulation: variable "%s" no reconocida, se omite.', var_name);
                    continue;
                end
                plot(log_t, logvars.(var_name), 'LineWidth', 1.0);
                if isKey(label_map, var_name)
                    legend_right{end+1} = label_map(var_name);
                else
                    legend_right{end+1} = strrep(var_name, '_', ' ');
                end
            end
            ylabel(strjoin(legend_right, ' / '));

            legend_entries = [legend_left, legend_right];
            if length(legend_entries) > 1
                legend(legend_entries, 'Location', 'eastoutside', 'FontSize', 7);
            end
            title(strjoin(legend_entries, ', '));

        else
            % --- Panel de eje único (como antes) ---
            vars_p = panel_p;
            legend_entries = {};

            for v = 1:length(vars_p)
                var_name = vars_p{v};
                if ~isfield(logvars, var_name)
                    warning('plot_simulation: variable "%s" no reconocida, se omite.', var_name);
                    continue;
                end
                plot(log_t, logvars.(var_name), 'Color', colors(v,:), 'LineWidth', 1.0);
                if isKey(label_map, var_name)
                    legend_entries{end+1} = label_map(var_name);
                else
                    legend_entries{end+1} = strrep(var_name, '_', ' ');
                end
                if strcmp(var_name, 'CL')
                    yline(CL_max, '--r', sprintf('CL_{max} = %.2f', CL_max), ...
                          'LineWidth', 1.0, 'LabelHorizontalAlignment', 'left', 'FontSize', 7);
                end
            end

            if length(vars_p) > 1
                legend(legend_entries, 'Location', 'eastoutside', 'FontSize', 7);
                ylabel(strjoin(legend_entries, ' / '));
            elseif ~isempty(legend_entries)
                ylabel(legend_entries{1});
            end

            title(strjoin(legend_entries, ', '));
        end
    end
    
        % --- Marcas de vuelta en todos los paneles ---
        for p = 1:n_panels
            for k = 1:length(t_marcas)
                xline(ax(p), t_marcas(k), '--', 'Color', [0.5 0.5 0.5], ...
                      'LineWidth', 0.7, 'Alpha', 0.6);
            end
        end
        for k = 1:length(t_marcas)
            xline(ax(1), t_marcas(k), '--', sprintf('V%d', k), ...
                  'Color', [0.5 0.5 0.5], 'LineWidth', 0.7, ...
                  'LabelOrientation', 'horizontal', ...
                  'LabelVerticalAlignment', 'bottom', 'FontSize', 7);
        end
    
        xlabel(ax(end), 'Tiempo [s]');
        linkaxes(ax, 'x');
    end

end