classdef (Abstract) BaseAero < handle
    properties
        RefArea         (1,1) double = 0.01966 % S_ref (m^2)
        CD0                           = 0.35    % Zero-lift drag: scalar, or a Mach-dependent struct (see evalCoeff)
        RefLength       (1,1) double = 0.155   % d_ref (m)
        Tables          struct = struct()      % Named coefficient sets, e.g. Tables.Clean.CD0/.CDA2/.CDA4/.CLA/.CLA3
        ActiveTableKey  (1,:) char = 'Clean'
        DeploymentRatio (1,1) double = 0.0
    end
    methods (Abstract)
        [F_aero, M_aero] = getForcesAndMoments(obj, t, X, env, fin_deflections)
    end
    methods
        function loadTable(obj, name, key)
            % Loads a named coefficient set from data/<name>.<ext> (any
            % format FactoryData understands - json/xml/fci/xlsx/mat/txt/
            % sldd) and stores it under obj.Tables.(key), then makes it
            % the active table. The loaded struct's fields (CD0, CDA2,
            % CLA, Cl_p, ...) are whatever the concrete Aero subclass's
            % getForcesAndMoments expects from Tables.(ActiveTableKey) -
            % this method doesn't validate shape, it just gets the data
            % from disk to that property.
            if nargin < 3 || isempty(key)
                key = 'Clean';
            end
            data = FactoryData.load(name);
            if isempty(data)
                error('BaseAero:loadTable', ...
                    'No data file found for "%s" (looked for data/%s.<ext>)', name, name);
            end
            obj.Tables.(key) = data;
            obj.ActiveTableKey = key;
        end
    end
    methods (Static)
        function val = evalCoeff(coeff, mach)
            % Evaluates one aero coefficient at a given Mach number. coeff is:
            %   - a plain scalar          -> returned unchanged (Mach-independent)
            %   - struct('Type','Table', 'Mach',[...], 'Value',[...])
            %       -> linearly interpolated, clamped to the table's Mach range
            %   - struct('Type','Polynomial', 'a0',.., 'a1',.., 'a2',.., ...)
            %       -> value = sum(a_i * mach^i) over whichever a-fields are present
            %   - struct('Type','PiecewisePolynomial', 'Mach',[...], 'Coeffs',[...], 'Order',[...], 'Boundary',...)
            %       -> Mach-regime table (McCoy/PRODAS-style, or NATO Fire
            %       Control Interface's segmented format): each row of
            %       Coeffs is a local polynomial a0/a1/.. (Order gives each
            %       column's power of mach) valid over one Mach segment;
            %       the query mach (clamped to the table's range, like
            %       Table above) picks its segment's row and is evaluated
            %       against it directly - no interpolation between rows.
            %       Boundary (default 'Lower') says which end of its
            %       segment each Mach value is: 'Lower' - Mach(i) is that
            %       segment's start (McCoy/PRODAS/text-table style, last
            %       breakpoint <= query wins); 'Upper' - Mach(i) is that
            %       segment's end (FCI's Maximum_Mach, first breakpoint
            %       >= query wins).
            if ~isstruct(coeff)
                val = coeff;
                return;
            end
            switch coeff.Type
                case 'Table'
                    m_clamped = min(max(mach, min(coeff.Mach)), max(coeff.Mach));
                    val = interp1(coeff.Mach, coeff.Value, m_clamped, 'linear');
                case 'Polynomial'
                    val = 0;
                    fn = fieldnames(coeff);
                    for i = 1:numel(fn)
                        tok = regexp(fn{i}, '^a(\d+)$', 'tokens', 'once');
                        if ~isempty(tok)
                            order = str2double(tok{1});
                            val = val + coeff.(fn{i}) * mach^order;
                        end
                    end
                case 'PiecewisePolynomial'
                    m_clamped = min(max(mach, min(coeff.Mach)), max(coeff.Mach));
                    if isfield(coeff, 'Boundary') && strcmpi(coeff.Boundary, 'Upper')
                        segIdx = find(coeff.Mach >= m_clamped, 1, 'first');
                    else
                        segIdx = find(coeff.Mach <= m_clamped, 1, 'last');
                    end
                    val = 0;
                    for k = 1:numel(coeff.Order)
                        val = val + coeff.Coeffs(segIdx, k) * m_clamped^coeff.Order(k);
                    end
                otherwise
                    error('BaseAero:evalCoeff', 'Unknown aero coefficient Type: %s', coeff.Type);
            end
        end

        function flow = computeFlowState(X, env)
            % Shared velocity/Mach/dynamic-pressure calculation every
            % concrete Aero (and DynamicsMPMM's yaw-of-repose) class needs -
            % reads density/speed-of-sound at the current altitude (X(3),
            % Z-up) from env rather than duplicating this per class.
            % vel_dir/gamma are zeroed when v_mag is ~0 rather than
            % producing NaN; callers still decide their own "too slow to
            % matter" early-return threshold.
            vel   = X(4:6);
            v_mag = norm(vel);
            altitude = X(3);

            flow.vel   = vel;
            flow.v_mag = v_mag;
            flow.mach  = v_mag / env.getSpeedOfSound(altitude);
            flow.q_bar = 0.5 * env.getDensity(altitude) * v_mag^2;

            if v_mag > 1e-3
                flow.vel_dir = vel / v_mag;
                flow.gamma   = atan2(vel(3), vel(1));
            else
                flow.vel_dir = [0; 0; 0];
                flow.gamma   = 0;
            end
        end
    end
end
