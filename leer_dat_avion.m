function T = leer_dat_avion(data_in)
%Carga el archivo .dat usando readtable y devuelve tabla limpia de cl vs cd
    if ischar(data_in) || isstring(data_in)
        try
            opts = detectImportOptions(data_in,'FileType','text');
            opts = setvartype(opts,'double'); %forzar columnas a datos numericos
            raw_matrix = readmatrix(data_in, opts);

            %ignoro la primera columna (velocidad)
            cl_data = raw_matrix{:,2};
            cd_data = raw_matrix{:,3};

        catch e 
            error ('Error al leer el archivo de polar %s %s', data_in, e.message)
        end
    else
        error('leer_dat_avion: La entrada debe ser el nombre de un archivo (string o char).');
    end

    [cl_sorted, sort_idx] = sort(cl_data);
    cd_sorted = cd_data(sort_idx);

    [unique_cl, ~, idx] = unique (cl_sorted);
    cd_mean = accumarray(idx, cd_sorted, [], @mean);

    %Final 
    T = table(unique_cl, cd_mean, 'VariableNames', {'cl','cd'});
end
