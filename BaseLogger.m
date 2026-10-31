classdef BaseLogger < handle
    properties
        LogToCommand (1,1) logical = true
        LogToFile    (1,1) logical = false
        FilePath     (1,:) char    = 'simulation_report.txt'
        FileID       (1,1) double  = -1
    end

    methods
        function obj = BaseLogger(to_cmd, to_file, filepath)
            % Defensively handle optional input arguments
            if nargin > 0 && ~isempty(to_cmd)
                obj.LogToCommand = to_cmd;
            end
            if nargin > 1 && ~isempty(to_file)
                obj.LogToFile = to_file;
            end
            
            if nargin > 2 && ~isempty(filepath)
                % Check if a directory path is already included in the filename
                [dir_path, ~, ~] = fileparts(filepath);
                
                if isempty(dir_path)
                    % Only a filename was given -> route to 'outputs/' folder
                    folder_name = 'outputs';
                    if ~isfolder(folder_name)
                        mkdir(folder_name);
                    end
                    obj.FilePath = fullfile(folder_name, filepath);
                else
                    % A full path or relative path with directories was given -> use as-is
                    obj.FilePath = filepath;
                    % Ensure target directory exists just in case
                    if ~isfolder(dir_path)
                        mkdir(dir_path);
                    end
                end
            end
            
            % Open file if logging to file is enabled
            if obj.LogToFile
                obj.FileID = fopen(obj.FilePath, 'w');
                if obj.FileID == -1
                    error('BaseLogger:FileOpen', 'Could not open file %s for writing.', obj.FilePath);
                end
            end
        end

        function delete(obj)
            if obj.FileID ~= -1
                fclose(obj.FileID);
            end
        end

        function write(obj, formatStr, varargin)
            msg = sprintf(formatStr, varargin{:});
            
            if obj.LogToCommand
                fprintf('%s', msg);
            end
            
            if obj.LogToFile && obj.FileID ~= -1
                fprintf(obj.FileID, '%s', msg);
            end
        end

        function writeLine(obj, formatStr, varargin)
            obj.write([formatStr, '\n'], varargin{:});
        end
    end
end