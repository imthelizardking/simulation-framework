classdef Aero6DoF < BaseAero
    % Aero model driven directly by Missile DATCOM's output tables.
    % Provides static forces (CD, CL, CY, CN, CA), moments (CM, CLN, CLL),
    % static stability derivatives, and dynamic rate damping derivatives.

    properties
        % Grid Breakpoints
        Mach  (1,:) double = []  % Mach breakpoints
        Alpha (1,:) double = []  % Alpha breakpoints, RADIANS (ParserDatcom converts from DATCOM's native degrees)

        % Wind & Body Static Coefficients: numel(Alpha)-by-numel(Mach)
        CD    double = []        % Drag coefficient (wind axis)
        CL    double = []        % Lift coefficient (wind axis)
        CY    double = []        % Side force coefficient (body/wind axis)
        CM    double = []        % Pitching moment coefficient
        CN    double = []        % Normal force coefficient (body axis)
        CA    double = []        % Axial force coefficient (body axis)
        CLN   double = []        % Yawing moment coefficient
        CLL   double = []        % Rolling moment coefficient
        XCP   double = []        % Center of pressure location (calibers from moment center)

        % Static Angle Derivatives (PER RADIAN, converted by ParserDatcom)
        CLA   double = []        % dCL/dalpha
        CMA   double = []        % dCM/dalpha (pitch stiffness)
        CYB   double = []        % dCY/dbeta (side force due to sideslip)
        CNB   double = []        % dCn/dbeta (yaw stiffness / weathercock)
        CLB   double = []        % dCl/dbeta (dihedral / roll due to sideslip)

        % Dynamic Pitch Damping Derivatives (PER RADIAN, non-dim by Q*LREF/2*V)
        CMQ   double = []        % Pitch damping moment derivative
        CNQ   double = []        % Normal force due to pitch rate
        CAQ   double = []        % Axial force due to pitch rate
        CMAD  double = []        % dCM/d(alphadot) lag derivative
        CNAD  double = []        % dCN/d(alphadot) lag derivative

        % Dynamic Yaw & Roll Damping Derivatives (PER RADIAN, non-dim by R,P*LREF/2*V)
        CLNR  double = []        % Yaw damping moment derivative
        CYR   double = []        % Side force due to yaw rate
        CLLR  double = []        % Roll due to yaw rate
        CLLP  double = []        % Roll damping derivative
        CLNP  double = []        % Yaw due to roll rate
        CYP   double = []        % Side force due to roll rate

        % Linear lift-curve slope for simplified control design
        CL_alpha (1,1) double = 3.0

        % Optional second configuration, stored in inherited Tables.
        % ActiveTableKey selects the initial configuration. With no named
        % tables, the original coefficient properties remain the default.
        DeployedTableKey (1,:) char = ''
        DeploymentTime (1,1) double {mustBeReal, mustBeNonnegative} = Inf
        DeploymentDuration (1,1) double {mustBeReal, mustBeNonnegative, mustBeFinite} = 0
    end

    methods
        function [F_aero, M_aero] = getForcesAndMoments(obj, t, X, env, fin_deflection)
            if nargin < 5 || isempty(fin_deflection)
                fin_deflection = 0.0;
            end

            pitch = X(8);
            flow = BaseAero.computeFlowState(X, env, obj.Frame);
            % 2. Full inertial relative velocity vector [m/s]
            v_rel_inertial = flow.v_mag * flow.vel_dir;
            % pitch = atan2(v_rel_inertial(3), v_rel_inertial(1));
            % disp(flow.gamma);
            if flow.v_mag < 1e-3
                F_aero = [0; 0; 0];
                M_aero = [0; 0; 0];
                return;
            end

            % 1. Extract quaternion and compute Body-to-Inertial DCM
            quat = X(18:21);
            R_i2b = Dynamics6DoF.quatToDCM(quat);

            % % 2. Full inertial relative velocity vector [m/s]
            % v_rel_inertial = flow.v_mag * flow.vel_dir;

            % 3. Rotate inertial wind into body axes: v_body = R_ib * v_inertial = R_bi' * v_inertial
            v_body = R_i2b * v_rel_inertial;

            u = v_body(1); % Forward (out the nose)
            v = v_body(2); % Right (starboard)
            w = v_body(3); % Down (belly)

            % 4. Aerodynamic angles
            % alpha = atan2(w, u) + fin_deflection;
            alpha = atan2(w, u);            
            % alpha = 0.0;
            % disp(strcat('pitch: ', num2str(pitch*180/pi)));
            % disp(strcat('phi: ', num2str(psi*180/pi)));
            % disp(strcat('psi: ', num2str(psi*180/pi)));
            % disp(strcat('alpha: ', num2str(alpha*180/pi)));
            % Extract sideslip angle (beta) and angular rates if available in state vector X
            % beta  = asin(v / max(flow.v_mag, 1e-3));
            if isfield(flow, 'beta')
                beta = flow.beta;
            elseif numel(X) >= 8 && isfield(flow, 'psi')
                % Estimate lateral sideslip if yaw/heading is present in state
                beta = X(8) - flow.psi;
            end
            beta = 0;
            % Angular rates P, Q, R (rad/s)
            p_rate = 0.0; q_rate = 0.0; r_rate = 0.0;
            if numel(X) >= 12
                p_rate = X(10);
                q_rate = X(11);
                r_rate = X(12);
            end

            % Interpolate static and damping coefficients at (Mach, Alpha)
            coeffs = obj.evaluateCoefficients(flow.mach, alpha, t);

            % 1. Aerodynamic Forces NATIVELY IN MISSILE BODY AXES
            % Coordinate definition for DATCOM body axes:
            %   X_b (Nose):   Axial force opposes forward flight -> -CA
            %   Y_b (Right):  Side force due to sideslip         -> +CY + CYB*beta
            %   Z_b (Belly):  Normal force opposes positive alpha -> -CN
            qS = flow.q_bar * obj.RefArea;

            Fx_b = -coeffs.CA * qS;
            Fy_b = (coeffs.CY + coeffs.CYB * beta) * qS;
            Fz_b = -coeffs.CN * qS;

            % This vector is strictly in BODY AXES (matching Dynamics6DoF's expectation)
            F_aero = [Fx_b; Fy_b; Fz_b];

            % 2. Aerodynamic Moments (Body Frame)
            % Reference dimensions
            s_ref = obj.RefArea;
            d_ref = obj.RefLength;
            q_bar = flow.q_bar;
            v_inf = flow.v_mag;

            % Non-dimensional angular rate factors
            q_hat = (q_rate * d_ref) / (2.0 * v_inf);
            r_hat = (r_rate * d_ref) / (2.0 * v_inf);
            p_hat = (p_rate * d_ref) / (2.0 * v_inf);

            % Total moment coefficients: static + sideslip buildup + dynamic damping
            Cl_total = coeffs.CLL + (coeffs.CLB * beta) + (coeffs.CLLP * p_hat) + (coeffs.CLLR * r_hat);
            Cm_total = coeffs.CM  + (coeffs.CMQ * q_hat);
            Cn_total = coeffs.CLN + (coeffs.CNB * beta) + (coeffs.CLNR * r_hat) + (coeffs.CLNP * p_hat);

            % Scale to dimensional moments: M = C * q_bar * S_ref * d_ref
            % Vector order: [Roll (Mx); Pitch (My); Yaw (Mz)]
            M_aero = (q_bar * s_ref * d_ref) * [Cl_total; Cm_total; Cn_total];
        end

        function setConfiguration(obj, key, source)
            % Register a snapshot of an Aero6DoF or a struct containing
            % Mach/Alpha and alpha-by-Mach coefficient matrices. All sets
            % must use this object's RefArea/RefLength and moment center.
            key = char(key);
            if ~isvarname(key)
                error('Aero6DoF:ConfigurationKey', 'Configuration key must be a valid field name.');
            end
            if ~(isa(source, 'Aero6DoF') || (isstruct(source) && isscalar(source)))
                error('Aero6DoF:ConfigurationData', 'Expected an Aero6DoF or scalar struct.');
            end
            for name = {'RefArea', 'RefLength'}
                f = name{1};
                if (isobject(source) || isfield(source, f)) && ~isequal(source.(f), obj.(f))
                    error('Aero6DoF:ReferenceMismatch', 'Configurations must share %s.', f);
                end
            end
            data = struct();
            for name = [{'Mach', 'Alpha'}, obj.coefficientFields()]
                f = name{1};
                if isobject(source) || isfield(source, f)
                    data.(f) = source.(f);
                end
            end
            obj.validateConfiguration(data);
            obj.Tables.(key) = data;
        end

        function ratio = deploymentRatioAt(obj, t)
            % Pure time lookup: RK4 substeps must not mutate deployment
            % state. Inf disables scheduling and uses DeploymentRatio.
            if nargin > 1
                validateattributes(t, {'double'}, {'scalar', 'real', 'finite'});
            end
            if nargin < 2 || isinf(obj.DeploymentTime)
                if ~isreal(obj.DeploymentRatio) || ~isfinite(obj.DeploymentRatio) || ...
                        obj.DeploymentRatio < 0 || obj.DeploymentRatio > 1
                    error('Aero6DoF:DeploymentRatio', 'DeploymentRatio must be between 0 and 1.');
                end
                ratio = obj.DeploymentRatio;
            elseif obj.DeploymentDuration == 0
                ratio = double(t >= obj.DeploymentTime);
            else
                ratio = min(1, max(0, (t - obj.DeploymentTime) / obj.DeploymentDuration));
            end
        end

        function coeffs = evaluateCoefficients(obj, mach, alpha, t)
            % Interpolate each configuration on its own grid, then blend
            % scalar coefficients. Two-argument calls use DeploymentRatio;
            % pass t for scheduled deployment (the force path does this).
            if nargin < 4
                ratio = obj.deploymentRatioAt();
            else
                ratio = obj.deploymentRatioAt(t);
            end
            if isempty(obj.DeployedTableKey)
                ratio = 0;
            end
            if ratio < 1
                initial = obj.configuration(obj.ActiveTableKey);
                coeffs = obj.evaluateTable(initial, mach, alpha);
            end
            if ratio > 0
                deployed = obj.configuration(obj.DeployedTableKey);
                final = obj.evaluateTable(deployed, mach, alpha);
                if ratio == 1
                    coeffs = final;
                else
                    for name = obj.coefficientFields()
                        f = name{1};
                        coeffs.(f) = (1 - ratio) * coeffs.(f) + ratio * final.(f);
                    end
                end
            end
        end

        function F_drag = dragEquation(obj, coeffs, flow)
            F_drag = -(flow.q_bar * coeffs.CD * obj.RefArea) * flow.vel_dir;
        end

        function F_lift = liftEquation(obj, coeffs, flow)
            lift_dir = cross(flow.vel_dir, CoordinateFrame.lateralAxis(obj.Frame));
            if norm(lift_dir) > 0
                lift_dir = lift_dir / norm(lift_dir);
            end
            F_lift = (flow.q_bar * coeffs.CL * obj.RefArea) * lift_dir;
        end

        function F_side = sideForceEquation(obj, coeffs, flow, beta)
            % Lateral unit vector in velocity coordinate system
            side_dir = CoordinateFrame.lateralAxis(obj.Frame);
            if norm(side_dir) > 0
                side_dir = side_dir / norm(side_dir);
            end
            % Total side force: CY baseline + CYB * beta
            CY_total = coeffs.CY + (coeffs.CYB * beta);
            F_side = (flow.q_bar * CY_total * obj.RefArea) * side_dir;
        end
    end

    methods (Access = private)
        function data = configuration(obj, key)
            if isfield(obj.Tables, key)
                data = obj.Tables.(key);
            elseif strcmp(key, 'Clean')
                data = obj; % Legacy ParserDatcom-loaded properties.
            else
                error('Aero6DoF:MissingConfiguration', 'Unknown configuration: %s.', key);
            end
        end
    end

    methods (Static, Access = private)
        function names = coefficientFields()
            names = {'CD','CL','CY','CM','CN','CA','CLN','CLL','XCP', ...
                'CLA','CMA','CYB','CNB','CLB','CMQ','CNQ','CAQ','CMAD', ...
                'CNAD','CLNR','CYR','CLLR','CLLP','CLNP','CYP'};
        end

        function validateConfiguration(data)
            for name = {'Mach', 'Alpha', 'CD', 'CL'}
                if ~isfield(data, name{1}) || isempty(data.(name{1}))
                    error('Aero6DoF:NoData', 'Configuration requires %s.', name{1});
                end
            end
            for name = {'Mach', 'Alpha'}
                grid = data.(name{1});
                validateattributes(grid, {'double'}, {'vector', 'real', 'finite', 'nonempty'});
                if any(diff(grid(:)) <= 0)
                    error('Aero6DoF:Grid', '%s must be strictly increasing.', name{1});
                end
            end
            expected = [numel(data.Alpha), numel(data.Mach)];
            for name = Aero6DoF.coefficientFields()
                f = name{1};
                if isfield(data, f) && ~isempty(data.(f))
                    if ~isequal(size(data.(f)), expected)
                        error('Aero6DoF:CoefficientSize', ...
                            '%s must have numel(Alpha) rows and numel(Mach) columns.', f);
                    end
                    validateattributes(data.(f), {'double'}, {'real', 'finite'});
                end
            end
        end

        function coeffs = evaluateTable(data, mach, alpha)
            if isempty(data.Mach) || isempty(data.Alpha)
                error('Aero6DoF:NoData', 'No aerodynamic configuration loaded.');
            end
            mc = min(max(mach, min(data.Mach)), max(data.Mach));
            ac = min(max(alpha, min(data.Alpha)), max(data.Alpha));
            for name = Aero6DoF.coefficientFields()
                f = name{1};
                coeffs.(f) = 0;
                if isstruct(data) && ~isfield(data, f), continue; end
                mat = data.(f);
                if isempty(mat), continue; end
                if isscalar(mat)
                    coeffs.(f) = mat;
                elseif numel(data.Mach) == 1
                    coeffs.(f) = interp1(data.Alpha, mat(:,1), ac, 'linear');
                elseif numel(data.Alpha) == 1
                    coeffs.(f) = interp1(data.Mach, mat(1,:), mc, 'linear');
                else
                    coeffs.(f) = interp2(data.Mach, data.Alpha, mat, mc, ac, 'linear');
                end
            end
        end
    end
end
