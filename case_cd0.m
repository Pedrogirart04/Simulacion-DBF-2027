function cd0_actual = case_cd0(fase, dt, V, CD0_TAKEOFF_TABLE)
    cd0_base   = 0.016; % Sin sensor / crucero
    cd0_sensor = 0.035; % Sensor 100% afuera

    switch fase
        case 'despegue' %analizar esto POSTERIORMENTEEEE????
            % Depende de la velocidad V (leyendo la tabla)
            cd0_actual = interp1(CD0_TAKEOFF_TABLE(:,1), CD0_TAKEOFF_TABLE(:,2), V, 'linear', 'extrap');
            
        case 'mision_sensor'
            % Depende del tiempo dt dentro de la pierna
            if dt < 4 
                cd0_actual = cd0_base;
            elseif dt >= 4 && dt < 7
                progreso = (dt - 4) / 3;
                cd0_actual = cd0_base + progreso * (cd0_sensor - cd0_base);
            elseif dt >= 7 && dt < 17
                cd0_actual = cd0_sensor;
            elseif dt >= 17 && dt < 20
                progreso = (dt - 17) / 3;
                cd0_actual = cd0_sensor - progreso * (cd0_sensor - cd0_base);
            else
                cd0_actual = cd0_base;
            end
            
        case 'crucero'
            cd0_actual = cd0_base;
    end
end


%Como funcionaria en el bucle de airplane_dynamics::
%% Pierna #2
%x0 = x; y0 = y;
%t_inicio_pierna = t; % Sacamos la foto al tiempo inicial de esta pierna

%while sqrt((x-x0)^2 + (y-y0)^2) < 130
    
    % 1. Calculamos el dt de esta pierna y la velocidad
    %dt = t - t_inicio_pierna;
    %V_inst = norm([v_x, v_y, v_z]);
    
    % 2. La función hace todo el trabajo sucio por detrás:
    %cd0_actual = calcular_cd0_instantaneo('mision_sensor', dt, V_inst, CD0_TAKEOFF_TABLE);
    
    % 3. Llamada a la simulación
    %[x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,inform,Energy,t] = ...
        %airplane_dynamics_opt(MTOW,t_n,ro,S_ref,x,y,z,v_x,v_y,v_z, ...
                              %roll_rad,pitch_rad,yaw_rad,throttle, cd0_actual, ...
                              %PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                              %inform,Energy,t,0,0);