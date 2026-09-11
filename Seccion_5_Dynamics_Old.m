%% ========================================================================
%  SECCIÓN 5: LOOP PRINCIPAL DE VUELTAS
%  ========================================================================

for vuelta = 1:n_vueltas

    t_inicio_vuelta = t;
    fprintf('--- Vuelta %d/%d (t = %.1f s) ---\n', vuelta, n_vueltas, t);

    % Heading acumulado base para esta vuelta
    heading_base = (vuelta - 1) * 2 * pi;

    % =====================================================================
    %  PIERNA 1 (ida)
    % =====================================================================
    x0 = x;  y0 = y;
    fprintf('  Pierna 1...');
    while sqrt((x - x0)^2 + (y - y0)^2) < largo_pierna(1) 
    % Cuando L_recorrido = sqrt(deltaX^2 + deltaY^2) < L_a_recorrer sigue el loop 

        % Chequeo de tiempo límite
        if t >= t_max
            mision_abortada = true;
            fprintf(' TIEMPO LÍMITE\n');
            break;
        end

        pitch_rad = 0;
        roll_rad  = 0;
        
        %Cinematica/Dinamica en t, Datos t (Input)-->Datos t+1 (Output)
        step_idx = step_idx + 1;
        [x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad, log_step, Energy,t] = ...
            airplane_dynamics_opt(MTOW, dt, rho, S_ref, ...
                x, y, z, v_x, v_y, v_z, roll_rad, pitch_rad, yaw_rad, ...
                throttle, cd0, ...
                PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                Energy, t, S_Banner, cd_Banner);

        inform(:, step_idx) = log_step;

    end
    if mision_abortada; break; end
    fprintf(' OK (%.1f m)\n', sqrt((x-x0)^2+(y-y0)^2));

    % =====================================================================
    %  GIRO 1 (180°)
    % =====================================================================
    heading_target = heading_base + heading_giro1;
    fprintf('  Giro 1...');

    % --- Fase A: Rolear hasta bank objetivo ---
    roll_objetivo  = -bank_rad;     % Negativo = giro a izquierda
    roll_rate = roll_objetivo / t_transicion;   % [rad/s]

    while abs(roll_rad - roll_objetivo) > deg2rad(2)
    %Hasta que Roll difiera del objetivo menos de 2 grados   
        if t >= t_max; mision_abortada = true; break; end
        pitch_rad = 0;
        roll_rad  = roll_rad + dt * roll_rate;

        % Clamp para no pasarse
        if roll_rate < 0
            roll_rad = max(roll_rad, roll_objetivo);
        else
            roll_rad = min(roll_rad, roll_objetivo);
        end

        %Cinematica/Dinamica en t, Datos t (Input)-->Datos t+1 (Output)
        step_idx = step_idx + 1;
        [x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad, log_step, Energy,t] = ...
            airplane_dynamics_opt(MTOW, dt, rho, S_ref, ...
                x, y, z, v_x, v_y, v_z, roll_rad, pitch_rad, yaw_rad, ...
                throttle, cd0, ...
                PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                Energy, t, S_Banner, cd_Banner);

        inform(:, step_idx) = log_step;
    end
    if mision_abortada; break; end

    % --- Fase B: Mantener bank hasta alcanzar heading ---
    while abs(yaw_rad - heading_target) > deg2rad(2)
    %Hasta que Yaw difiera del objetivo menos de 2 grados 
        if t >= t_max; mision_abortada = true; break; end
        pitch_rad = 0;

        %Cinematica/Dinamica en t, Datos t (Input)-->Datos t+1 (Output)
        step_idx = step_idx + 1;
        [x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad, log_step, Energy,t] = ...
            airplane_dynamics_opt(MTOW, dt, rho, S_ref, ...
                x, y, z, v_x, v_y, v_z, roll_rad, pitch_rad, yaw_rad, ...
                throttle, cd0, ...
                PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                Energy, t, S_Banner, cd_Banner);

        inform(:, step_idx) = log_step;
    end
    if mision_abortada; break; end

    % --- Fase C: Nivelar (volver a roll = 0) ---
    roll_rate_nivelar = -roll_objetivo / t_transicion;  % signo opuesto

    while abs(roll_rad) > deg2rad(2)
        if t >= t_max; mision_abortada = true; break; end
        pitch_rad = 0;
        roll_rad  = roll_rad + dt * roll_rate_nivelar;

        % Clamp para no pasarse de 0
        if roll_rate_nivelar > 0
            roll_rad = min(roll_rad, 0);
        else
            roll_rad = max(roll_rad, 0);
        end

        %Cinematica/Dinamica en t, Datos t (Input)-->Datos t+1 (Output)
        step_idx = step_idx + 1;
        [x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad, log_step, Energy,t] = ...
            airplane_dynamics_opt(MTOW, dt, rho, S_ref, ...
                x, y, z, v_x, v_y, v_z, roll_rad, pitch_rad, yaw_rad, ...
                throttle, cd0, ...
                PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                Energy, t, S_Banner, cd_Banner);

        inform(:, step_idx) = log_step;
    end
    if mision_abortada; break; end
    roll_rad = 0;
    yaw_rad = heading_base + deg2rad(180);   % Snap heading al valor exacto
    fprintf(' OK (heading = %.1f°)\n', rad2deg(yaw_rad));

    % =====================================================================
    %  PIERNA 2 (vuelta)
    % =====================================================================
    x0 = x;  y0 = y;
    fprintf('  Pierna 2...');

    while sqrt((x - x0)^2 + (y - y0)^2) < largo_pierna(2)
        if t >= t_max; mision_abortada = true; break; end
        pitch_rad = 0;
        roll_rad  = 0;

        %Cinematica/Dinamica en t, Datos t (Input)-->Datos t+1 (Output)
        step_idx = step_idx + 1;
        [x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad, log_step, Energy,t] = ...
            airplane_dynamics_opt(MTOW, dt, rho, S_ref, ...
                x, y, z, v_x, v_y, v_z, roll_rad, pitch_rad, yaw_rad, ...
                throttle, cd0, ...
                PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                Energy, t, S_Banner, cd_Banner);

        inform(:, step_idx) = log_step;
    end
    if mision_abortada; break; end
    fprintf(' OK (%.1f m)\n', sqrt((x-x0)^2+(y-y0)^2));
    

        % =====================================================================
    %  GIRO 2 (180°, vuelve al heading original)
    % =====================================================================
    heading_target = heading_base + heading_giro2;
    fprintf('  Giro 2...');

    % --- Fase A: Rolear ---
    roll_objetivo  = -bank_rad;
    roll_rate = roll_objetivo / t_transicion;

    while abs(roll_rad - roll_objetivo) > deg2rad(2)
        if t >= t_max; mision_abortada = true; break; end
        pitch_rad = 0;
        roll_rad  = roll_rad + dt * roll_rate;
        if roll_rate < 0
            roll_rad = max(roll_rad, roll_objetivo);
        else
            roll_rad = min(roll_rad, roll_objetivo);
        end

        %Cinematica/Dinamica en t, Datos t (Input)-->Datos t+1 (Output)
        step_idx = step_idx + 1;
        [x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad, log_step, Energy,t] = ...
            airplane_dynamics_opt(MTOW, dt, rho, S_ref, ...
                x, y, z, v_x, v_y, v_z, roll_rad, pitch_rad, yaw_rad, ...
                throttle, cd0, ...
                PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                Energy, t, S_Banner, cd_Banner);

        inform(:, step_idx) = log_step;
    end
    if mision_abortada; break; end

    % --- Fase B: Mantener bank ---
    while abs(yaw_rad - heading_target) > deg2rad(2)
        if t >= t_max; mision_abortada = true; break; end
        pitch_rad = 0;

        %Cinematica/Dinamica en t, Datos t (Input)-->Datos t+1 (Output)
        step_idx = step_idx + 1;
        [x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad, log_step, Energy,t] = ...
            airplane_dynamics_opt(MTOW, dt, rho, S_ref, ...
                x, y, z, v_x, v_y, v_z, roll_rad, pitch_rad, yaw_rad, ...
                throttle, cd0, ...
                PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                Energy, t, S_Banner, cd_Banner);

        inform(:, step_idx) = log_step;
    end
    if mision_abortada; break; end

    % --- Fase C: Nivelar ---
    roll_rate_nivelar = -roll_objetivo / t_transicion;

    while abs(roll_rad) > deg2rad(2)
        if t >= t_max; mision_abortada = true; break; end
        pitch_rad = 0;
        roll_rad  = roll_rad + dt * roll_rate_nivelar;
        if roll_rate_nivelar > 0
            roll_rad = min(roll_rad, 0);
        else
            roll_rad = max(roll_rad, 0);
        end

        %Cinematica/Dinamica en t, Datos t (Input)-->Datos t+1 (Output)
        step_idx = step_idx + 1;
        [x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad, log_step, Energy,t] = ...
            airplane_dynamics_opt(MTOW, dt, rho, S_ref, ...
                x, y, z, v_x, v_y, v_z, roll_rad, pitch_rad, yaw_rad, ...
                throttle, cd0, ...
                PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                Energy, t, S_Banner, cd_Banner);

        inform(:, step_idx) = log_step;
    end
    if mision_abortada; break; end
    roll_rad = 0;
    yaw_rad = heading_base + deg2rad(360);   % Snap heading al valor exacto
    %VER
    fprintf(' OK (heading = %.1f°)\n', rad2deg(yaw_rad));

    % =====================================================================
    %  FIN DE VUELTA
    % =====================================================================
    t_por_vuelta(vuelta) = t - t_inicio_vuelta;
    vueltas_completadas = vuelta;

    fprintf('  Vuelta %d completada en %.2f s (E = %.3f Ah)\n\n', ...
            vuelta, t_por_vuelta(vuelta), Energy);
end

% Resumen post-loop
fprintf('=== SIMULACIÓN TERMINADA ===\n');
if mision_abortada
    fprintf('Misión abortada: ');
    if t >= t_max
        fprintf('tiempo límite alcanzado (%.0f s)\n', t_max);
    end
end
fprintf('Vueltas completadas: %d/%d\n', vueltas_completadas, n_vueltas);
fprintf('Tiempo total: %.2f s\n', t);
fprintf('Energía total: %.3f Ah\n', Energy);
if vueltas_completadas > 0
    fprintf('Tiempo promedio por vuelta: %.2f s\n', mean(t_por_vuelta(1:vueltas_completadas)));
end
fprintf('\n');

inform = inform(:, 1:step_idx);
