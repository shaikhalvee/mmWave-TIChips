function [file_name] = GET_ADC_DATA_BIN_FILE(varargin)
    if nargin ~= 0
        file_name = varargin{1};
    else
        [file, path] = uigetfile('*.bin','Select adc_data.bin file');
        if isequal(file,0)
            fprintf('User selected Cancel \n');
            error('no .bin file selected')
        else
            fprintf('User selected %s \n',fullfile(path,file));
            file_name = fullfile(path, file);
        end
    end
end
