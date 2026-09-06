function T = leer_dat_mot(data_in)

% Lee un archivo .dat y devuelve la tabla (T)


%% --- Bloque 1: Cargar/Asignar la tabla T ---
    if ischar(data_in) || isstring(data_in)
        % Opción A: data_in es un nombre de archivo, lo leemos.
        filename = data_in; 
        try
            opts = detectImportOptions(filename, 'FileType','text'); % Detecta el archivo que subis
            opts = setvartype(opts, 'double'); % Setea las columnas como numeros 
            T = readtable(filename, opts); % Lee el archivo y arma la tabla
        catch e
            error('Error al leer el archivo de motor %s: %s', filename, e.message);
        end
    else 
        error('dat_mot: la entrada debe ser un nombre de archivo (string o char).');
    end 
end