classdef DatcomReader
    % Reads a Missile DATCOM printed-output listing (data/<name>.txt - the
    % standard line-printer text DATCOM writes, e.g. for006.dat/*.out) and
    % normalizes its "CHARACTERISTICS AT ANGLE OF ATTACK" tables into an
    % AeroDatcom object.
    %
    % Format confirmed against a real Digital DATCOM run: data/ex1_in.txt,
    % "Example Problem 1" from the DATCOM Users Manual (data/EX1_inout.txt
    % is that run's INPUT deck, for reference). DATCOM does NOT write a
    % MATLAB script - its native output is this fixed-format text listing,
    % with one "CHARACTERISTICS AT ANGLE OF ATTACK AND IN SIDESLIP" block
    % per Mach/case, each block holding a Mach number and a table of
    %   ALPHA  CD  CL  CM  CN  CA  XCP  CLA  CMA  CYB  CNB  CLB
    % rows (CD/CL already wind-axis, computed by DATCOM itself - no
    % body->wind transform needed on this side, unlike the CN/CA-only
    % assumption an earlier version of this reader made before a real
    % sample file was available to check against).
    %
    % A case with no method available for a condition prints "NDM" (No
    % DATCOM Methods) in place of numbers for that whole Mach block; such
    % blocks are skipped rather than fabricating a coefficient. If the
    % same Mach number appears in more than one case (a deck commonly
    % refines the same flight condition across CASEID/NEXT CASE blocks,
    % e.g. an axisymmetric-body pass followed by a cambered-body pass at
    % the same Mach), the LAST such block in the file wins - it represents
    % the deck's most-refined configuration at that condition.
    methods (Static)
        function aero = load(name, ref_area, ref_length)
            here = fileparts(mfilename('fullpath'));
            path = fullfile(here, 'data', [name '.txt']);
            if ~isfile(path)
                error('DatcomReader:load', 'No DATCOM output found at data/%s.txt', name);
            end

            blocks = DatcomReader.parseBlocks(path);
            if isempty(blocks)
                error('DatcomReader:load', ...
                    ['data/%s.txt has no usable "CHARACTERISTICS AT ANGLE OF ATTACK" ' ...
                     'table (missing, all NDM, or an unrecognized layout).'], name);
            end

            % Canonical alpha schedule: DATCOM keeps ALSCHD fixed across
            % NEXT CASE blocks unless a case explicitly overrides it, so
            % every block should share it. Blocks that don't match the
            % first block's schedule are dropped rather than misaligned
            % against it.
            alpha_deg = blocks(1).alpha_deg;
            keep = true(1, numel(blocks));
            for i = 2:numel(blocks)
                if numel(blocks(i).alpha_deg) ~= numel(alpha_deg) || ...
                        max(abs(blocks(i).alpha_deg - alpha_deg)) > 1e-6
                    keep(i) = false;
                end
            end
            blocks = blocks(keep);
            if isempty(blocks)
                error('DatcomReader:load', ...
                    'data/%s.txt has no blocks sharing a common ALPHA schedule.', name);
            end

            mach_list = [blocks.mach];
            [mach_sorted, order] = sort(mach_list); % stable: ties keep file order
            blocks = blocks(order);
            mach_unique = unique(mach_sorted, 'stable');
            n_mach  = numel(mach_unique);
            n_alpha = numel(alpha_deg);
            CD = zeros(n_alpha, n_mach);
            CL = zeros(n_alpha, n_mach);
            for i = 1:numel(blocks)
                col = find(mach_unique == blocks(i).mach, 1);
                CD(:, col) = blocks(i).CD(:); % later (higher i) blocks overwrite earlier duplicates
                CL(:, col) = blocks(i).CL(:);
            end
            [mach_unique, order2] = sort(mach_unique);
            CD = CD(:, order2);
            CL = CL(:, order2);

            aero = AeroDatcom();
            if nargin > 1 && ~isempty(ref_area),   aero.RefArea   = ref_area;   end
            if nargin > 2 && ~isempty(ref_length), aero.RefLength = ref_length; end

            aero.Mach  = mach_unique;
            aero.Alpha = deg2rad(alpha_deg(:)');
            aero.CD    = CD;
            aero.CL    = CL;
            aero.CL_alpha = DatcomReader.estimateCLAlpha(aero);
        end
    end

    methods (Static, Access = private)
        function blocks = parseBlocks(path)
            raw = fileread(path);
            lines = strsplit(raw, {'\r\n', '\n'});
            blocks = struct('mach', {}, 'alpha_deg', {}, 'CD', {}, 'CL', {});

            title_idx = find(contains(lines, 'CHARACTERISTICS AT ANGLE OF ATTACK'));
            for k = 1:numel(title_idx)
                i0 = title_idx(k);
                i1 = min(i0 + 80, numel(lines));
                window = lines(i0:i1);

                mach = DatcomReader.extractMach(window);
                if isempty(mach), continue; end

                hdr = find(contains(window, '0 ALPHA') & contains(window, ' CD ') & contains(window, ' CN '), 1);
                if isempty(hdr), continue; end

                alpha_deg = [];
                cd_vals = [];
                cl_vals = [];
                valid = true;
                for j = (hdr + 1):numel(window)
                    line = window{j};
                    if ~isempty(line) && line(1) == '1'
                        break; % line-printer top-of-page: end of this block
                    end
                    m = regexp(line, '^\s*(-?\d+\.\d+)\s+(.*)$', 'tokens', 'once');
                    if isempty(m)
                        continue; % blank / carriage-control-only line between rows
                    end
                    a = str2double(m{1});
                    rest = strsplit(strtrim(m{2}));
                    if any(strcmpi(rest, 'NDM'))
                        valid = false;
                        break;
                    end
                    if numel(rest) < 2
                        valid = false;
                        break;
                    end
                    cd_v = str2double(rest{1});
                    cl_v = str2double(rest{2});
                    if isnan(cd_v) || isnan(cl_v)
                        valid = false;
                        break;
                    end
                    alpha_deg(end + 1) = a; %#ok<AGROW>
                    cd_vals(end + 1) = cd_v; %#ok<AGROW>
                    cl_vals(end + 1) = cl_v; %#ok<AGROW>
                end

                if valid && ~isempty(alpha_deg)
                    blocks(end + 1) = struct('mach', mach, 'alpha_deg', alpha_deg, ...
                        'CD', cd_vals, 'CL', cl_vals); %#ok<AGROW>
                end
            end
        end

        function mach = extractMach(window)
            % The Mach number sits on the flight-conditions data line a
            % few lines above the ALPHA table header, identified by its
            % leading "0" carriage-control column and a scientific-
            % notation Reynolds number later on the same line, e.g.:
            %   0 0.600                                       4.2800E+06  8.850  2.480 ...
            mach = [];
            for j = 1:numel(window)
                m = regexp(window{j}, '^0\s+(\d+\.\d+)\s+.*\d+\.\d+E[+-]\d+', 'tokens', 'once');
                if ~isempty(m)
                    mach = str2double(m{1});
                    return;
                end
            end
        end

        function cla = estimateCLAlpha(aero)
            % Representative linear lift-curve slope for controllers that
            % need a single gain (see AeroDatcom.CL_alpha) - the CL slope
            % at the smallest nonzero alpha breakpoint, averaged across
            % Mach.
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
