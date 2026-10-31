classdef ParserCustom
    % PARSERCUSTOM Reads structured CSV or TXT tabular aerodynamic files
    % containing column headers and maps them into an Aero6DoF object.

    methods (Static)
        function aero = load(filename, ref_area, ref_length)
            path = ParserCustom.resolveFilePath(filename);
            
            % Read tabular data with header names
            opts = detectImportOptions(path);
            data_table = readtable(path, opts);
            
            required_cols = {'Mach', 'Alpha', 'CD', 'CL', 'CM'};
            for c = 1:numel(required_cols)
                if ~ismember(required_cols{c}, data_table.Properties.VariableNames)
                    error('ParserCustom:MissingColumn', 'Custom table missing required column: %s', required_cols{c});
                end
            end

            mach_unique  = sort(unique(data_table.Mach));
            alpha_unique = sort(unique(data_table.Alpha));
            n_mach  = numel(mach_unique);
            n_alpha = numel(alpha_unique);

            CD  = zeros(n_alpha, n_mach);
            CL  = zeros(n_alpha, n_mach);
            CM  = zeros(n_alpha, n_mach);
            CN  = zeros(n_alpha, n_mach);
            CA  = zeros(n_alpha, n_mach);
            CY  = zeros(n_alpha, n_mach);
            CLN = zeros(n_alpha, n_mach);
            CLL = zeros(n_alpha, n_mach);
            XCP = zeros(n_alpha, n_mach);

            for i = 1:height(data_table)
                m_val = data_table.Mach(i);
                a_val = data_table.Alpha(i);
                
                i_a = find(abs(alpha_unique - a_val) < 1e-4, 1);
                i_m = find(abs(mach_unique  - m_val) < 1e-4, 1);

                if isempty(i_a) || isempty(i_m), continue; end

                CD(i_a, i_m) = data_table.CD(i);
                CL(i_a, i_m) = data_table.CL(i);
                CM(i_a, i_m) = data_table.CM(i);
                
                if ismember('CN', data_table.Properties.VariableNames), CN(i_a, i_m) = data_table.CN(i); end
                if ismember('CA', data_table.Properties.VariableNames), CA(i_a, i_m) = data_table.CA(i); end
                if ismember('CY', data_table.Properties.VariableNames), CY(i_a, i_m) = data_table.CY(i); end
                if ismember('CLN', data_table.Properties.VariableNames), CLN(i_a, i_m) = data_table.CLN(i); end
                if ismember('CLL', data_table.Properties.VariableNames), CLL(i_a, i_m) = data_table.CLL(i); end
                if ismember('XCP', data_table.Properties.VariableNames), XCP(i_a, i_m) = data_table.XCP(i); end
            end

            aero = Aero6DoF();
            if nargin > 1 && ~isempty(ref_area),   aero.RefArea   = ref_area;   end
            if nargin > 2 && ~isempty(ref_length), aero.RefLength = ref_length; end

            aero.Mach  = mach_unique(:)';
            aero.Alpha = deg2rad(alpha_unique(:)'); % Assuming input alpha is degrees, converted to radians

            aero.CD  = CD;
            aero.CL  = CL;
            aero.CM  = CM;
            aero.CN  = CN;
            aero.CA  = CA;
            aero.CY  = CY;
            aero.CLN = CLN;
            aero.CLL = CLL;
            aero.XCP = XCP;
        end
    end

    methods (Static, Access = private)
        function path = resolveFilePath(name)
            if isfile(name), path = name; return; end
            here = fileparts(mfilename('fullpath'));
            cand1 = fullfile(here, 'data', [name '.csv']);
            cand2 = fullfile(here, 'data', [name '.txt']);
            cand3 = fullfile(here, 'data', name);
            if isfile(cand1), path = cand1;
            elseif isfile(cand2), path = cand2;
            elseif isfile(cand3), path = cand3;
            else, error('ParserCustom:resolveFilePath', 'Could not locate file: %s', name);
            end
        end
    end
end