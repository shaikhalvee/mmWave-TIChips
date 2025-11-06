function [file_name] = GET_MMWAVE_SETUP_JSON_FILE(varargin)
    if nargin ~= 0
        file_name = varargin{1};
    else
        [file, path] = uigetfile('*.JSON','Select mmwave_setup JSON file');
        if isequal(file,0)
             fprintf('User selected Cancel \n');
             error('no .JSON file selected')
        else
            fprintf('User selected %s \n',fullfile(path,file));
            file_name = fullfile(path, file);
        end
    end
end
