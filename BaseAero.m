classdef (Abstract) BaseAero < handle
    properties
        RefArea         (1,1) double = 0.01966 % S_ref (m^2)
        CD0                           = 0.35    % Zero-lift drag: scalar, or a Mach-dependent struct (see evalCoeff)
        RefLength       (1,1) double = 0.155   % d_ref (m)
        Tables          struct = struct()      % Named coefficient sets, e.g. Tables.Clean.CD0/.CDA2/.CDA4/.CLA/.CLA3
        ActiveTableKey  (1,:) char = 'Clean'
        DeploymentRatio (1,1) double = 0.0
        MomentReference = [] % optional 3x1 body-datum position (m); [] uses legacy CGRef
        Frame           (1,:) char = 'Z-Up'    % 'Z-Up' or 'NED' - see CoordinateFrame; must match the attached Dynamics

        % STANAG 4355 fitting/form factors - generic drag/lift tuning
        % multipliers against real firing-table data (see MunitionData.
        % formFactor, Munition.applyFormFactors). Neutral (1.0) reproduces
        % whatever this class's own force equations already compute
        % unchanged. AeroMPMM applies these ITSELF, internally, using its
        % own true alpha_e-relative drag/lift terms (see that class) -
        % every OTHER concrete Aero class gets them applied generically
        % instead, by Munition.getDerivatives calling
        % BaseAero.applyFormFactors on the returned F_aero: since every
        % other class here builds F_aero as exactly (drag along
        % -velocity) + (lift perpendicular to velocity), that force
        % vector can be split back into those two parts from the velocity
        % direction alone and scaled independently - exact, not an
        % approximation, for any class following that convention. A class
        % with no lift term (e.g. AeroParticle) is unaffected by FL, since
        % its perpendicular-to-velocity component is already zero.
        IForm (1,1) double = 1.0   % drag-component multiplier
        FL    (1,1) double = 1.0   % lift-component multiplier
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

            % Real external data doesn't always tag a Type - default to
            % PiecewisePolynomial (the McCoy/PRODAS/FCI-style per-Mach-
            % segment shape) when it's absent, since that's the format
            % most likely to arrive untagged.
            if isfield(coeff, 'Type')
                switch_type = coeff.Type;
            else
                switch_type = 'PiecewisePolynomial';
            end

            switch switch_type
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

                    % Order defaults to sequential powers 1..N (one per
                    % Coeffs column) when not given explicitly.
                    if isfield(coeff, 'Order')
                        order_ = coeff.Order;
                    else
                        order_ = 1:size(coeff.Coeffs, 2);
                    end

                    for k = 1:numel(order_)
                        val = val + coeff.Coeffs(segIdx, k) * m_clamped^order_(k);
                    end
                otherwise
                    error('BaseAero:evalCoeff', 'Unknown aero coefficient Type: %s', coeff.Type);
            end
        end

        function flow = computeFlowState(X, env, frame)
            % Shared velocity/Mach/dynamic-pressure calculation every
            % concrete Aero (and DynamicsMPMM's yaw-of-repose) class needs -
            % reads density/speed-of-sound at the current altitude from env
            % rather than duplicating this per class. frame ('Z-Up'
            % default, 'NED', or 'FDR' - see CoordinateFrame) only affects
            % how altitude is read back out of X; vel/gamma need no special
            % handling because X is already expressed consistently in
            % that same frame (see CoordinateFrame's class comment).
            % vel_dir/gamma are zeroed when v_mag is ~0 rather than
            % producing NaN; callers still decide their own "too slow to
            % matter" early-return threshold.
            if nargin < 3, frame = 'Z-Up'; end
            vel   = X(4:6);
            v_mag = norm(vel);
            altitude = CoordinateFrame.altitude(X, frame);
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

        function F_scaled = applyFormFactors(F_aero, vel, IForm, FL)
            % Generic post-hoc IForm/FL scaling for any Aero class that
            % builds F_aero as (drag along -velocity) + (lift
            % perpendicular to velocity) - see the IForm/FL property
            % comment for why this split is exact, not approximate, for
            % that convention. Called by Munition.getDerivatives for
            % every Aero class except AeroMPMM (which applies its own
            % IForm/FL internally already - calling this too would square
            % the effect). A no-op (IForm=FL=1, or v_mag~0) returns F_aero
            % unchanged.
            v_mag = norm(vel);
            if v_mag < 1e-6 || (IForm == 1.0 && FL == 1.0)
                F_scaled = F_aero;
                return;
            end
            vel_dir = vel / v_mag;
            F_drag_component = dot(F_aero, vel_dir) * vel_dir; % component along the velocity axis (drag is normally negative here)
            F_lift_component = F_aero - F_drag_component;      % remainder, perpendicular to velocity by construction
            F_scaled = IForm * F_drag_component + FL * F_lift_component;
        end
    end
end
