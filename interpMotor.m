function filaInterp = interpMotor(T, throttle_query, extrapol)

    if nargin < 3 
        extrapol = false; 
    end

    if  ~istable(T) % Chequeo que T sea una tabla
        error('La entrada de "motor" debe ser una tabla (table).');
    end

     if ~ismember('ESC_throttle', T.Properties.VariableNames) % Chequeo que este la columna de Throttle
        error('interpMotor: la tabla no tiene una columna ''ESC_throttle''.');
     end

     throttle_min = min(T.ESC_throttle); % Throttle maximo de la tabla
     throttle_max = max(T.ESC_throttle); % Throttle minimo de la tabla

     if ~extrapol % Si puso que no extrapola y en realidad es necesario, tira error
        if throttle_query < throttle_min || throttle_query > throttle_max
            error('interpMotor: throttle_query (%g) fuera de rango [%g, %g].', ...
                throttle_query, throttle_min, throttle_max);
        end
     end
            campos = T.Properties.VariableNames; % nombres de columnas
            filaInterp = struct();
            
            % Interpolar o extrapolar todas las columnas excepto ESC_throttle
            for i = 1:numel(campos)
                campo = campos{i};
                if strcmp(campo,'ESC_throttle') % Saltea cuando la columna es de throttle
                    continue
                end 
               try 
                filaInterp.(campo) = interp1(T.ESC_throttle, T.(campo), throttle_query, 'linear', 'extrap'); 
               catch e
                   warning('interpMotor: error interpolando campo %s: %s', campo, e.message);
                   filaInterp.(campo) = NaN;
               end
            end 
            
            % Agregar el valor de throttle consultado
            filaInterp.ESC_throttle = throttle_query;
        


end

