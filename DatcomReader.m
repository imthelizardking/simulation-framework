classdef DatcomReader
    % DATCOMREADER Reads USAF Automated Missile DATCOM (REV 03/11) formatted
    % output listings (e.g. for006.dat, data/<name>.txt, data/<name>.dat)
    % and normalizes static forces, moments, stability derivatives, and dynamic
    % rate damping derivatives into an AeroDatcom object or a struct.
    %
    % Angular derivatives (CNA, CMA, CYB, CLNB, CLLB) and dynamic rate
    % derivatives (CNQ, CMQ, CAQ, CNAD, CMAD, CYR, CLNR, CLLR, CYP, CLNP, CLLP)
    % are converted from DATCOM's native [1/deg] to standard [1/rad].

    methods (Static)
        function aero = load(name, ref_area, ref_length)
            % Resolve input file path
            path = DatcomReader.resolveFilePath(name);

            % Parse raw data into unified record struct
            records = DatcomReader.parseRawDatcom(path);
            if isempty(records)
                error('DatcomReader:load', 'No valid aerodynamic records found in %s', path);
            end

            % Extract canonical Alpha and Mach schedules
            all_mach  = [records.Mach];
            all_alpha = [records.Alpha];
            mach_unique  = sort(unique(all_mach));
            alpha_unique = sort(unique(all_alpha));

            n_mach  = numel(mach_unique);
            n_alpha = numel(alpha_unique);

            % Allocate 2D matrices [n_alpha x n_mach] to match AeroDatcom layout
            CD   = zeros(n_alpha, n_mach);
            CL   = zeros(n_alpha, n_mach);
            CM   = zeros(n_alpha, n_mach);
            CN   = zeros(n_alpha, n_mach);
            CA   = zeros(n_alpha, n_mach);
            CY   = zeros(n_alpha, n_mach);
            CLN  = zeros(n_alpha, n_mach);
            CLL  = zeros(n_alpha, n_mach);
            XCP  = zeros(n_alpha, n_mach);
            CLA  = zeros(n_alpha, n_mach);
            CMA  = zeros(n_alpha, n_mach);
            CYB  = zeros(n_alpha, n_mach);
            CNB  = zeros(n_alpha, n_mach);
            CLB  = zeros(n_alpha, n_mach);
            CMQ  = zeros(n_alpha, n_mach);
            CNQ  = zeros(n_alpha, n_mach);
            CAQ  = zeros(n_alpha, n_mach);
            CMAD = zeros(n_alpha, n_mach);
            CNAD = zeros(n_alpha, n_mach);
            CLNR = zeros(n_alpha, n_mach);
            CYR  = zeros(n_alpha, n_mach);
            CLLR = zeros(n_alpha, n_mach);
            CLLP = zeros(n_alpha, n_mach);
            CLNP = zeros(n_alpha, n_mach);
            CYP  = zeros(n_alpha, n_mach);

            rad_factor = 180 / pi;

            % Populate grid
            for k = 1:numel(records)
                r = records(k);
                i_a = find(abs(alpha_unique - r.Alpha) < 1e-4, 1);
                i_m = find(abs(mach_unique  - r.Mach)  < 1e-4, 1);

                if isempty(i_a) || isempty(i_m), continue; end

                CD(i_a, i_m)   = r.CD;
                CL(i_a, i_m)   = r.CL;
                CM(i_a, i_m)   = r.CM;
                CN(i_a, i_m)   = r.CN;
                CA(i_a, i_m)   = r.CA;
                CY(i_a, i_m)   = r.CY;
                CLN(i_a, i_m)  = r.CLN;
                CLL(i_a, i_m)  = r.CLL;
                XCP(i_a, i_m)  = r.XCP;

                % Convert static angle derivatives from 1/deg to 1/rad
                CLA(i_a, i_m)  = r.CNA * rad_factor; % Normal-force slope ≈ lift slope
                CMA(i_a, i_m)  = r.CMA * rad_factor;
                CYB(i_a, i_m)  = r.CYB * rad_factor;
                CNB(i_a, i_m)  = r.CLNB * rad_factor;
                CLB(i_a, i_m)  = r.CLLB * rad_factor;

                % Dynamic rate damping derivatives (already in 1/rad from parseRawDatcom)
                CMQ(i_a, i_m)  = r.CMQ;
                CNQ(i_a, i_m)  = r.CNQ;
                CAQ(i_a, i_m)  = r.CAQ;
                CMAD(i_a, i_m) = r.CMAD;
                CNAD(i_a, i_m) = r.CNAD;
                CLNR(i_a, i_m) = r.CLNR;
                CYR(i_a, i_m)  = r.CYR;
                CLLR(i_a, i_m) = r.CLLR;
                CLLP(i_a, i_m) = r.CLLP;
                CLNP(i_a, i_m) = r.CLNP;
                CYP(i_a, i_m)  = r.CYP;
            end

            % Instantiate AeroDatcom object
            aero = AeroDatcom();
            if nargin > 1 && ~isempty(ref_area),   aero.RefArea   = ref_area;   end
            if nargin > 2 && ~isempty(ref_length), aero.RefLength = ref_length; end

            aero.Mach  = mach_unique(:)';
            aero.Alpha = deg2rad(alpha_unique(:)'); % Stored in radians

            aero.CD   = CD;
            aero.CL   = CL;
            aero.CM   = CM;
            aero.CN   = CN;
            aero.CA   = CA;
            aero.CY   = CY;
            aero.CLN  = CLN;
            aero.CLL  = CLL;
            aero.XCP  = XCP;
            aero.CLA  = CLA;
            aero.CMA  = CMA;
            aero.CYB  = CYB;
            aero.CNB  = CNB;
            aero.CLB  = CLB;

            % Dynamic damping matrices
            if isprop(aero, 'CMQ'),  aero.CMQ  = CMQ;  end
            if isprop(aero, 'CNQ'),  aero.CNQ  = CNQ;  end
            if isprop(aero, 'CAQ'),  aero.CAQ  = CAQ;  end
            if isprop(aero, 'CMAD'), aero.CMAD = CMAD; end
            if isprop(aero, 'CNAD'), aero.CNAD = CNAD; end
            if isprop(aero, 'CLNR'), aero.CLNR = CLNR; end
            if isprop(aero, 'CYR'),  aero.CYR  = CYR;  end
            if isprop(aero, 'CLLP'), aero.CLLP = CLLP; end
            if isprop(aero, 'CLLR'), aero.CLLR = CLLR; end

            if isprop(aero, 'CL_alpha')
                aero.CL_alpha = DatcomReader.estimateCLAlpha(aero);
            end
        end

        function aero_struct = loadStruct(filename)
            % LOADSTRUCT Direct extraction returning a struct with 3-column
            % [Mach, Alpha, Value] matrices for every coefficient.
            if nargin < 1
                filename = 'for006.dat';
            end
            path = DatcomReader.resolveFilePath(filename);
            records = DatcomReader.parseRawDatcom(path);

            all_fields = {'CN','CM','CA','CY','CLN','CLL','CL','CD','XCP', ...
                          'CNA','CMA','CYB','CLNB','CLLB', ...
                          'CNQ','CMQ','CAQ','CNAD','CMAD','CYR','CLNR','CLLR','CYP','CLNP','CLLP'};

            num_pts = numel(records);
            mach_vec  = [records.Mach]';
            alpha_vec = [records.Alpha]';

            [~, sort_idx] = sortrows([mach_vec, alpha_vec], [1, 2]);

            aero_struct = struct();
            for f = 1:length(all_fields)
                fn = all_fields{f};
                val_vec = [records.(fn)]';
                aero_struct.(fn) = [mach_vec(sort_idx), alpha_vec(sort_idx), val_vec(sort_idx)];
            end

            aero_struct.MachGrid  = unique(mach_vec)';
            aero_struct.AlphaGrid = unique(alpha_vec)';
        end
    end

    methods (Static, Access = private)
        function path = resolveFilePath(name)
            if isfile(name)
                path = name;
                return;
            end
            here = fileparts(mfilename('fullpath'));
            cand1 = fullfile(here, 'data', [name '.txt']);
            cand2 = fullfile(here, 'data', [name '.dat']);
            cand3 = fullfile(here, 'data', name);
            if isfile(cand1)
                path = cand1;
            elseif isfile(cand2)
                path = cand2;
            elseif isfile(cand3)
                path = cand3;
            else
                error('DatcomReader:resolveFilePath', 'Could not locate file: %s', name);
            end
        end

        function records = parseRawDatcom(filepath)
            fid = fopen(filepath, 'r');
            if fid == -1
                error('DatcomReader:parseRawDatcom', 'Cannot open file: %s', filepath);
            end
            raw_text = fread(fid, '*char')';
            fclose(fid);

            lines = regexp(raw_text, '\r?\n', 'split');
            num_lines = length(lines);

            all_fields = {'CN','CM','CA','CY','CLN','CLL','CL','CD','XCP', ...
                          'CNA','CMA','CYB','CLNB','CLLB', ...
                          'CNQ','CMQ','CAQ','CNAD','CMAD','CYR','CLNR','CLLR','CYP','CLNP','CLLP'};

            dynamic_fields = {'CNQ','CMQ','CAQ','CNAD','CMAD','CYR','CLNR','CLLR','CYP','CLNP','CLLP'};

            data_map = containers.Map();
            current_mach = NaN;
            rad_factor = 180 / pi;
            i = 1;

            while i <= num_lines
                line_str = strtrim(lines{i});

                % 1. Detect Mach Number
                if contains(line_str, 'MACH NO') && contains(line_str, '=')
                    tok = regexp(line_str, 'MACH\s+NO\s*=\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)', 'tokens');
                    if ~isempty(tok)
                        current_mach = str2double(tok{1}{1});
                    end
                    i = i + 1;
                    continue;
                end

                % 2. Detect Table Headers containing ALPHA
                if ~isnan(current_mach) && (startsWith(line_str, 'ALPHA') || contains(line_str, ' ALPHA '))
                    clean_hdr = regexprep(line_str, 'X-C\.P\.', 'XCP');
                    tokens = regexp(clean_hdr, '\S+', 'match');

                    matched_cols = [];
                    matched_names = {};
                    for k = 2:length(tokens)
                        c_name = tokens{k};
                        if ismember(c_name, all_fields)
                            matched_cols(end+1) = k; %#ok<AGROW>
                            matched_names{end+1} = c_name; %#ok<AGROW>
                        end
                    end

                    if ~isempty(matched_cols)
                        i = i + 1;
                        while i <= num_lines
                            row_str = strtrim(lines{i});

                            % Skip blank lines between header and numbers
                            if isempty(row_str)
                                i = i + 1;
                                continue;
                            end

                            % Termination conditions for subtable
                            if contains(row_str, '---') || contains(row_str, '*****') || ...
                               contains(row_str, 'PAGE') || contains(row_str, 'PANEL') || ...
                               contains(row_str, 'DERIVATIVE') || contains(row_str, 'FLIGHT CONDITIONS')
                                break;
                            end

                            nums = sscanf(row_str, '%f');
                            if length(nums) >= 2 && ~isnan(nums(1))
                                a_val = nums(1);
                                key = sprintf('%.4f_%.4f', current_mach, a_val);

                                if ~isKey(data_map, key)
                                    entry = struct('Mach', current_mach, 'Alpha', a_val);
                                    for f = 1:length(all_fields)
                                        entry.(all_fields{f}) = NaN;
                                    end
                                    data_map(key) = entry;
                                end

                                entry = data_map(key);
                                for m = 1:length(matched_names)
                                    col_idx = matched_cols(m);
                                    if col_idx <= length(nums)
                                        val = nums(col_idx);
                                        % Convert dynamic damping terms to 1/rad
                                        if ismember(matched_names{m}, dynamic_fields) && ~isnan(val)
                                            val = val * rad_factor;
                                        end
                                        entry.(matched_names{m}) = val;
                                    end
                                end
                                data_map(key) = entry;
                            else
                                break;
                            end
                            i = i + 1;
                        end
                        continue;
                    end
                end
                i = i + 1;
            end

            keys_list = keys(data_map);
            N = length(keys_list);
            if N == 0
                records = [];
                return;
            end

            row_cells = cell(N, 1);
            for k = 1:N
                row_cells{k} = data_map(keys_list{k});
            end
            records = cell2mat(row_cells);
        end

        function cla = estimateCLAlpha(aero)
            nonzero = find(aero.Alpha > 0, 1, 'first');
            if isempty(nonzero)
                cla = 3.0;
                return;
            end
            a = aero.Alpha(nonzero);
            cla = mean(aero.CL(nonzero, :)) / a;
        end
    end
end