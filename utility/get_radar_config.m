function paramsConfig = get_radar_config(angles, adc_data_folder)
    paramsConfig = struct;
    paramsConfig.anglesToSteer = angles;
    paramsConfig.NumAnglesToSweep = length(angles);
    paramsConfig.Chirp_Frame_BF = 0;
    % matfile = fullfile(paramsFile);
    % if isfile(matfile)
    %     S = load(matfile);
    %     paramsConfig = S.params;
    %     fprintf('[Config] Loaded radar params from MAT file.\n');
    % else
        % fallback: parse from JSON as before (old data)
        paramsConfig = parameter_gen_from_Jason(adc_data_folder, paramsConfig);
        fprintf('[Config] Parsed radar params from JSON file.\n');
    % end
end
