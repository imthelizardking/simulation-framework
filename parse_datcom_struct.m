function aero = parse_datcom_struct(filename)
% PARSE_DATCOM_STRUCT Parses for006.dat into a struct of [Mach, Alpha, Coeff] matrices.
%
% Usage:
%   aero = parse_datcom_struct('for006.dat');

    if nargin < 1
        filename = 'for006.dat';
    end

    fid = fopen(filename, 'r');
    if fid == -1
        error('Cannot open file: %s', filename);
    end
    raw_lines = textscan(fid, '%s', 'Delimiter', '\n', 'Whitespace', '');
    fclose(fid);
    lines = raw_lines{1};
    num_lines = length(lines);

    all_fields = {'CN','CM','CA','CY','CLN','CLL','CL','CD','XCP', ...
                  'CNA','CMA','CYB','CLNB','CLLB', ...
                  'CNQ','CMQ','CAQ','CNAD','CMAD','CYR','CLNR','CLLR','CYP','CLNP','CLLP'};

    dynamic_fields = {'CNQ','CMQ','CAQ','CNAD','CMAD','CYR','CLNR','CLLR','CYP','CLNP','CLLP'};

    % Master map keyed by 'Mach_Alpha' to assemble everything
    data_map = containers.Map();
    current_mach = NaN;
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
            % Normalize header
            clean_hdr = regexprep(line_str, 'X-C\.P\.', 'XCP');
            tokens = regexp(clean_hdr, '\S+', 'match');

            % Verify this line actually has coefficient headers
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
                % Advance to numeric data rows (skipping any blank lines between header & data)
                i = i + 1;
                while i <= num_lines
                    row_str = strtrim(lines{i});

                    % Skip blank lines if encountered before or between data
                    if isempty(row_str)
                        i = i + 1;
                        continue;
                    end

                    % Stop reading if a page break, divider, or next header appears
                    if contains(row_str, '---') || contains(row_str, '*****') || ...
                       contains(row_str, 'PAGE') || contains(row_str, 'PANEL') || ...
                       contains(row_str, 'DERIVATIVE') || contains(row_str, 'FLIGHT CONDITIONS')
                        break;
                    end

                    nums = sscanf(row_str, '%f');
                    if length(nums) >= 2 && ~isnan(nums(1))
                        alpha_val = nums(1);
                        key = sprintf('%.4f_%.4f', current_mach, alpha_val);

                        if ~isKey(data_map, key)
                            entry = struct('Mach', current_mach, 'Alpha', alpha_val);
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
                                % Convert dynamic derivatives to 1/rad
                                if ismember(matched_names{m}, dynamic_fields) && ~isnan(val)
                                    val = val * (180 / pi);
                                end
                                entry.(matched_names{m}) = val;
                            end
                        end
                        data_map(key) = entry;
                    else
                        % If line cannot be parsed into numbers, table has ended
                        break;
                    end
                    i = i + 1;
                end
                continue;
            end
        end
        i = i + 1;
    end

    if isempty(data_map)
        error('No aerodynamic tables could be parsed from %s', filename);
    end

    % 3. Build struct with 3-column matrices: [Mach, Alpha, Coeff]
    keys_list = keys(data_map);
    num_pts = length(keys_list);

    mach_vec  = zeros(num_pts, 1);
    alpha_vec = zeros(num_pts, 1);
    for k = 1:num_pts
        ent = data_map(keys_list{k});
        mach_vec(k)  = ent.Mach;
        alpha_vec(k) = ent.Alpha;
    end

    % Sort by Mach ascending, then Alpha ascending
    [~, sort_idx] = sortrows([mach_vec, alpha_vec], [1, 2]);

    aero = struct();
    for f = 1:length(all_fields)
        cname = all_fields{f};
        val_vec = zeros(num_pts, 1);
        for k = 1:num_pts
            ent = data_map(keys_list{k});
            val_vec(k) = ent.(cname);
        end
        % Output [Mach, Alpha, Coeff]
        aero.(cname) = [mach_vec(sort_idx), alpha_vec(sort_idx), val_vec(sort_idx)];
    end

    aero.MachGrid  = unique(mach_vec)';
    aero.AlphaGrid = unique(alpha_vec)';

    fprintf('[SUCCESS] Loaded aero struct across %d Mach numbers.\n', length(aero.MachGrid));
    fprintf('          Dynamic damping derivatives scaled to [1/rad].\n');
end