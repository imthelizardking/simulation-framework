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
    % sample file was available to check against). All 11 non-ALPHA
    % columns are captured onto the resulting AeroDatcom; CLA/CMA/CYB/CNB/
    % CLB are DATCOM's own derivatives, tabulated PER DEGREE (see the
    % table's "-------DERIVATIVE (PER DEGREE)-------" banner in the raw
    % listing) - convert to per-radian before mixing with this framework's
    % own per-radian coefficients (e.g. DynamicsMPMM.CLalpha).
    %
    % This table alone does not cover everything Missile DATCOM can print
    % (dynamic/rate derivatives like CNQ/CMQ/CLP from a $DAMP request, or
    % control-surface effectiveness from $CONTROL, live in separate table
    % formats). Those aren't parsed here because no real sample DATCOM run
    % requesting them exists yet in data/ to verify a parser against - see
    % the note above about not fabricating a layout without a genuine
    % sample. Add a parser for them once such a run is available.
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
            CD  = zeros(n_alpha, n_mach);
            CL  = zeros(n_alpha, n_mach);
            CM  = zeros(n_alpha, n_mach);
            CN  = zeros(n_alpha, n_mach);
            CA  = zeros(n_alpha, n_mach);
            XCP = zeros(n_alpha, n_mach);
            CLA = zeros(n_alpha, n_mach);
            CMA = zeros(n_alpha, n_mach);
            CYB = zeros(n_alpha, n_mach);
            CNB = zeros(n_alpha, n_mach);
            CLB = zeros(n_alpha, n_mach);
            for i = 1:numel(blocks)
                col = find(mach_unique == blocks(i).mach, 1);
                CD(:, col)  = blocks(i).CD(:); % later (higher i) blocks overwrite earlier duplicates
                CL(:, col)  = blocks(i).CL(:);
                CM(:, col)  = blocks(i).CM(:);
                CN(:, col)  = blocks(i).CN(:);
                CA(:, col)  = blocks(i).CA(:);
                XCP(:, col) = blocks(i).XCP(:);
                CLA(:, col) = blocks(i).CLA(:);
                CMA(:, col) = blocks(i).CMA(:);
                CYB(:, col) = blocks(i).CYB(:);
                CNB(:, col) = blocks(i).CNB(:);
                CLB(:, col) = blocks(i).CLB(:);
            end
            [mach_unique, order2] = sort(mach_unique);
            CD  = CD(:, order2);
            CL  = CL(:, order2);
            CM  = CM(:, order2);
            CN  = CN(:, order2);
            CA  = CA(:, order2);
            XCP = XCP(:, order2);
            CLA = CLA(:, order2);
            CMA = CMA(:, order2);
            CYB = CYB(:, order2);
            CNB = CNB(:, order2);
            CLB = CLB(:, order2);

            aero = AeroDatcom();
            if nargin > 1 && ~isempty(ref_area),   aero.RefArea   = ref_area;   end
            if nargin > 2 && ~isempty(ref_length), aero.RefLength = ref_length; end

            aero.Mach  = mach_unique;
            aero.Alpha = deg2rad(alpha_deg(:)');
            aero.CD    = CD;
            aero.CL    = CL;
            aero.CM    = CM;
            aero.CN    = CN;
            aero.CA    = CA;
            aero.XCP   = XCP;
            aero.CLA   = CLA;
            aero.CMA   = CMA;
            aero.CYB   = CYB;
            aero.CNB   = CNB;
            aero.CLB   = CLB;
            aero.CL_alpha = DatcomReader.estimateCLAlpha(aero);
        end
    end

    methods (Static, Access = private)
        function blocks = parseBlocks(path)
            raw = fileread(path);
            lines = strsplit(raw, {'\r\n', '\n'});
            blocks = struct('mach', {}, 'alpha_deg', {}, 'CD', {}, 'CL', {}, ...
                'CM', {}, 'CN', {}, 'CA', {}, 'XCP', {}, 'CLA', {}, 'CMA', {}, ...
                'CYB', {}, 'CNB', {}, 'CLB', {});

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
                cd_vals  = [];
                cl_vals  = [];
                cm_vals  = [];
                cn_vals  = [];
                ca_vals  = [];
                xcp_vals = [];
                cla_vals = [];
                cma_vals = [];
                cyb_vals = [];
                cnb_vals = [];
                clb_vals = [];
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
                    % Fixed DATCOM column layout after ALPHA: CD CL CM CN
                    % CA XCP CLA CMA CYB CNB CLB (see class header comment)
                    % - all 11 should be present on a non-NDM row.
                    if numel(rest) < 11
                        valid = false;
                        break;
                    end
                    cd_v = str2double(rest{1});
                    cl_v = str2double(rest{2});
                    cm_v = str2double(rest{3});
                    cn_v = str2double(rest{4});
                    ca_v = str2double(rest{5});
                    % XCP = CM/CN is undefined wherever CN crosses zero
                    % (typically alpha=0); DATCOM prints "******" there
                    % instead of a number, which str2double turns into NaN
                    % - a per-point gap, not a reason to drop the whole
                    % block the way a NDM row or a bad CD/CL would.
                    xcp_v = str2double(rest{6});
                    cla_v = str2double(rest{7});
                    cma_v = str2double(rest{8});
                    cyb_v = str2double(rest{9});
                    cnb_v = str2double(rest{10});
                    clb_v = str2double(rest{11});
                    if isnan(cd_v) || isnan(cl_v)
                        valid = false;
                        break;
                    end
                    alpha_deg(end + 1) = a; %#ok<AGROW>
                    cd_vals(end + 1)  = cd_v; %#ok<AGROW>
                    cl_vals(end + 1)  = cl_v; %#ok<AGROW>
                    cm_vals(end + 1)  = cm_v; %#ok<AGROW>
                    cn_vals(end + 1)  = cn_v; %#ok<AGROW>
                    ca_vals(end + 1)  = ca_v; %#ok<AGROW>
                    xcp_vals(end + 1) = xcp_v; %#ok<AGROW>
                    cla_vals(end + 1) = cla_v; %#ok<AGROW>
                    cma_vals(end + 1) = cma_v; %#ok<AGROW>
                    cyb_vals(end + 1) = cyb_v; %#ok<AGROW>
                    cnb_vals(end + 1) = cnb_v; %#ok<AGROW>
                    clb_vals(end + 1) = clb_v; %#ok<AGROW>
                end

                if valid && ~isempty(alpha_deg)
                    blocks(end + 1) = struct('mach', mach, 'alpha_deg', alpha_deg, ...
                        'CD', cd_vals, 'CL', cl_vals, 'CM', cm_vals, ...
                        'CN', cn_vals, 'CA', ca_vals, 'XCP', xcp_vals, ...
                        'CLA', cla_vals, 'CMA', cma_vals, 'CYB', cyb_vals, ...
                        'CNB', cnb_vals, 'CLB', clb_vals); %#ok<AGROW>
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
