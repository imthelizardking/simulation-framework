classdef AeroDatcom < BaseAero
    % Aero model driven directly by Missile DATCOM's output tables.
    % Provides static forces (CD, CL, CY, CN, CA), moments (CM, CLN, CLL),
    % static stability derivatives, and dynamic rate damping derivatives.

    properties
        % Grid Breakpoints
        Mach  (1,:) double = []  % Mach breakpoints
        Alpha (1,:) double = []  % Alpha breakpoints, RADIANS (DatcomReader converts from DATCOM's native degrees)

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

        % Static Angle Derivatives (PER RADIAN, converted by DatcomReader)
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
    end

    methods
        function [F_aero, M_aero] = getForcesAndMoments(obj, t, X, env, fin_deflection)
            if nargin < 5 || isempty(fin_deflection)
                fin_deflection = 0.0;
            end

            pitch = X(8);
            flow = BaseAero.computeFlowState(X, env, obj.Frame);
            % disp(pitch);
            % disp(flow.gamma);
            if flow.v_mag < 1e-3
                F_aero = [0; 0; 0];
                M_aero = [0; 0; 0];
                return;
            end

            if isempty(obj.Mach) || isempty(obj.Alpha)
                error('AeroDatcom:NoData', ...
                    '%s has no DATCOM table loaded - see DatcomReader.load.', class(obj));
            end

            % 1. Extract quaternion and compute Body-to-Inertial DCM
            quat = X(18:21);
            R_bi = Dynamics6DoF.quatToDCM(quat);

            % 2. Full inertial relative velocity vector [m/s]
            v_rel_inertial = flow.v_mag * flow.vel_dir;

            % 3. Rotate inertial wind into body axes: v_body = R_ib * v_inertial = R_bi' * v_inertial
            v_body = R_bi' * v_rel_inertial;

            u = v_body(1); % Forward (out the nose)
            v = v_body(2); % Right (starboard)
            w = v_body(3); % Down (belly)

            % 4. Aerodynamic angles
            alpha = atan2(w, u) + fin_deflection;
            % alpha = (pitch - flow.gamma) + fin_deflection;             % Aerodynamic angle of attack including fin deflection / control bias % --- OLD (Breaks as gamma drops): ---
            % alpha = 0.0;
            % disp(alpha);
            % Extract sideslip angle (beta) and angular rates if available in state vector X
            % beta  = asin(v / max(flow.v_mag, 1e-3));
            beta = 0.0;
            if isfield(flow, 'beta')
                beta = flow.beta;
            elseif numel(X) >= 8 && isfield(flow, 'psi')
                % Estimate lateral sideslip if yaw/heading is present in state
                beta = X(8) - flow.psi;
            end

            % Angular rates P, Q, R (rad/s)
            p_rate = 0.0; q_rate = 0.0; r_rate = 0.0;
            if numel(X) >= 12
                p_rate = X(10);
                q_rate = X(11);
                r_rate = X(12);
            end

            % Interpolate static and damping coefficients at (Mach, Alpha)
            coeffs = obj.evaluateCoefficients(flow.mach, alpha);

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

        function coeffs = evaluateCoefficients(obj, mach, alpha)
            % Clamped interpolation matching DATCOM table bounds
            mach_c  = min(max(mach,  min(obj.Mach)),  max(obj.Mach));
            alpha_c = min(max(alpha, min(obj.Alpha)), max(obj.Alpha));

            % 2D Bilinear Interpolation
            coeffs.CD  = interp2(obj.Mach, obj.Alpha, obj.CD,  mach_c, alpha_c, 'linear');
            coeffs.CL  = interp2(obj.Mach, obj.Alpha, obj.CL,  mach_c, alpha_c, 'linear');
            coeffs.CY  = obj.interpField(obj.CY,  mach_c, alpha_c, 0.0);
            coeffs.CM  = obj.interpField(obj.CM,  mach_c, alpha_c, 0.0);
            coeffs.CN  = obj.interpField(obj.CN,  mach_c, alpha_c, 0.0);
            coeffs.CA  = obj.interpField(obj.CA,  mach_c, alpha_c, 0.0);
            coeffs.CLN = obj.interpField(obj.CLN, mach_c, alpha_c, 0.0);
            coeffs.CLL = obj.interpField(obj.CLL, mach_c, alpha_c, 0.0);

            % Static angle derivatives
            coeffs.CYB = obj.interpField(obj.CYB, mach_c, alpha_c, 0.0);
            coeffs.CNB = obj.interpField(obj.CNB, mach_c, alpha_c, 0.0);
            coeffs.CLB = obj.interpField(obj.CLB, mach_c, alpha_c, 0.0);

            % Dynamic damping derivatives
            coeffs.CMQ  = obj.interpField(obj.CMQ,  mach_c, alpha_c, 0.0);
            coeffs.CNQ  = obj.interpField(obj.CNQ,  mach_c, alpha_c, 0.0);
            coeffs.CLNR = obj.interpField(obj.CLNR, mach_c, alpha_c, 0.0);
            coeffs.CLLP = obj.interpField(obj.CLLP, mach_c, alpha_c, 0.0);
            coeffs.CLLR = obj.interpField(obj.CLLR, mach_c, alpha_c, 0.0);
            coeffs.CLNP = obj.interpField(obj.CLNP, mach_c, alpha_c, 0.0);
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
        function val = interpField(obj, mat, mach_c, alpha_c, default_val)
            if isempty(mat) || all(mat(:) == 0)
                val = default_val;
            else
                val = interp2(obj.Mach, obj.Alpha, mat, mach_c, alpha_c, 'linear');
            end
        end
    end
end