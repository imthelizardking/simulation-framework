classdef ParserProdas
    % PARSERPRODAS Reads PRODAS output reports or tabular text files
    % and normalizes aerodynamic coefficients into an Aero6DoF object.

    methods (Static)
        function aero = load(name, ref_area, ref_length)
            path = ParserProdas.resolveFilePath(name);
            records = ParserProdas.parseRawProdas(path);
            
            if isempty(records)
                error('ParserProdas:load', 'No valid aerodynamic records found in %s', path);
            end

            all_mach  = [records.Mach];
            all_alpha = [records.Alpha];
            mach_unique  = sort(unique(all_mach));
            alpha_unique = sort(unique(all_alpha));

            n_mach  = numel(mach_unique);
            n_alpha = numel(alpha_unique);

            % Initialize grid matrices [n_alpha x n_mach]
            CD  = zeros(n_alpha, n_mach);
            CL  = zeros(n_alpha, n_mach);
            CM  = zeros(n_alpha, n_mach);
            CN  = zeros(n_alpha, n_mach);
            CA  = zeros(n_alpha, n_mach);
            CY  = zeros(n_alpha, n_mach);
            CLN = zeros(n_alpha, n_mach);
            CLL = zeros(n_alpha, n_mach);
            XCP = zeros(n_alpha, n_mach);
            CLA = zeros(n_alpha, n_mach);
            CMA = zeros(n_alpha, n_mach);
            CYB = zeros(n_alpha, n_mach);
            CNB = zeros(n_alpha, n_mach);
            CLB = zeros(n_alpha, n_mach);

            rad_factor = 180 / pi;

            for k = 1:numel(records)
                r = records(k);
                i_a = find(abs(alpha_unique - r.Alpha) < 1e-4, 1);
                i_m = find(abs(mach_unique  - r.Mach)  < 1e-4, 1);

                if isempty(i_a) || isempty(i_m), continue; end

                CD(i_a, i_m)  = r.CD;
                CL(i_a, i_m)  = r.CL;
                CM(i_a, i_m)  = r.CM;
                CN(i_a, i_m)  = r.CN;
                CA(i_a, i_m)  = r.CA;
                CY(i_a, i_m)  = r.CY;
                CLN(i_a, i_m) = r.CLN;
                CLL(i_a, i_m) = r.CLL;
                XCP(i_a, i_m) = r.XCP;

                if isfield(r, 'CLA') && ~isnan(r.CLA), CLA(i_a, i_m) = r.CLA * rad_factor; end
                if isfield(r, 'CMA') && ~isnan(r.CMA), CMA(i_a, i_m) = r.CMA * rad_factor; end
                if isfield(r, 'CYB') && ~isnan(r.CYB), CYB(i_a, i_m) = r.CYB * rad_factor; end
                if isfield(r, 'CNB') && ~isnan(r.CNB), CNB(i_a, i_m) = r.CNB * rad_factor; end
                if isfield(r, 'CLB') && ~isnan(r.CLB), CLB(i_a, i_m) = r.CLB * rad_factor; end
            end

            aero = Aero6DoF();
            if nargin > 1 && ~isempty(ref_area),   aero.RefArea   = ref_area;   end
            if nargin > 2 && ~isempty(ref_length), aero.RefLength = ref_length; end

            aero.Mach  = mach_unique(:)';
            aero.Alpha = deg2rad(alpha_unique(:)'); % Stored in radians

            aero.CD  = CD;
            aero.CL  = CL;
            aero.CM  = CM;
            aero.CN  = CN;
            aero.CA  = CA;
            aero.CY  = CY;
            aero.CLN = CLN;
            aero.CLL = CLL;
            aero.XCP = XCP;
            if ~all(CLA(:) == 0), aero.CLA = CLA; end
            if ~all(CMA(:) == 0), aero.CMA = CMA; end
            if ~all(CYB(:) == 0), aero.CYB = CYB; end
            if ~all(CNB(:) == 0), aero.CNB = CNB; end
            if ~all(CLB(:) == 0), aero.CLB = CLB; end
        end
    end

    methods (Static, Access = private)
        function path = resolveFilePath(name)
            if isfile(name), path = name; return; end
            here = fileparts(mfilename('fullpath'));
            cand1 = fullfile(here, 'data', [name '.txt']);
            cand2 = fullfile(here, 'data', [name '.dat']);
            cand3 = fullfile(here, 'data', name);
            if isfile(cand1), path = cand1;
            elseif isfile(cand2), path = cand2;
            elseif isfile(cand3), path = cand3;
            else, error('ParserProdas:resolveFilePath', 'Could not locate file: %s', name);
            end
        end

        function records = parseRawProdas(filepath)
            % Robust line-by-line tabular search for PRODAS format
            fid = fopen(filepath, 'r');
            if fid == -1, error('ParserProdas:open', 'Cannot open %s', filepath); end
            raw_text = fread(fid, '*char')';
            fclose(fid);

            lines = regexp(raw_text, '\r?\n', 'split');
            records = struct('Mach', {}, 'Alpha', {}, 'CD', {}, 'CL', {}, 'CM', {}, 'CN', {}, 'CA', {}, 'CY', {}, 'CLN', {}, 'CLL', {}, 'XCP', {});
            
            % Generic fallback parser assuming space/tab delimited columns: [Mach, Alpha, CD, CL, CM, ...]
            for i = 1:numel(lines)
                nums = sscanf(strtrim(lines{i}), '%f');
                if numel(nums) >= 5 % Minimum: Mach, Alpha, CD, CL, CM
                    records(end+1) = struct( ...
                        'Mach', nums(1), 'Alpha', nums(2), ...
                        'CD', nums(3), 'CL', nums(4), 'CM', nums(5), ...
                        'CN', 0, 'CA', nums(3), 'CY', 0, 'CLN', 0, 'CLL', 0, 'XCP', 0);
                end
            end
        end
    end
end