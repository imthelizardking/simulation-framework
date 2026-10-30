classdef Aero6DoF < BaseAero
    % Aero model driven directly by Missile DATCOM's output tables.
    % Provides static forces (CD, CL, CY, CN, CA), moments (CM, CLN, CLL),
    % static stability derivatives, dynamic rate damping derivatives,
    % and control surface effectiveness derivatives.

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

        % --- NEW: Control Effectiveness Derivatives (PER RADIAN deflection) ---
        CND   double = []        % dCN/d(delta_q) - Pitch control normal force
        CMD   double = []        % dCM/d(delta_q) - Pitch control pitching moment
        CYD   double = []        % dCY/d(delta_r) - Yaw control side force
        CLND  double = []        % dCLN/d(delta_r) - Yaw control yawing moment
        CLLD  double = []        % dCLL/d(delta_p) - Roll control rolling moment

        % Linear lift-curve slope for simplified control design
        CL_alpha (1,1) double = 3.0
    end

    methods
        function [F_aero, M_aero] = getForcesAndMoments(obj, t, X, env, fin_deflection)
            if nargin < 5 || isempty(fin_deflection)
                fin_deflection = [0.0; 0.0; 0.0];
            end

            % Parse control deflections for 3 axes (Pitch, Yaw, Roll)
            % Assumes scalar input is purely pitch (elevator) deflection
            if numel(fin_deflection) >= 3
                del_q = fin_deflection(1); % Pitch (Elevator)
                del_r = fin_deflection(2); % Yaw (Rudder)
                del_p = fin_deflection(3); % Roll (Aileron)
            else
                del_q = fin_deflection(1);
                del_r = 0.0;
                del_p = 0.0;
            end

            flow = BaseAero.computeFlowState(X, env, obj.Frame);
            v_rel_inertial = flow.v_mag * flow.vel_dir;

            if flow.v_mag < 1e-3
                F_aero = [0; 0; 0];
                M_aero = [0; 0; 0];
                return;
            end

            if isempty(obj.Mach) || isempty(obj.Alpha)
                error('Aero6DoF:NoData', ...
                    '%s has no DATCOM table loaded - see ParserDatcom.load.', class(obj));
            end

            % 1. Extract quaternion, NORMALIZE IT, and compute Body-to-Inertial DCM
            quat = X(18:21);
            if norm(quat) > 0
                quat = quat / norm(quat); % Prevent matrix drift from scaling velocity
            end
            R_b2i = Dynamics6DoF.quatToDCM(quat);

            % 3. Rotate inertial wind into body axes
            v_body = R_b2i' * v_rel_inertial;
            u = v_body(1); % Forward (out the nose)
            v = v_body(2); % Right (starboard)
            w = v_body(3); % Down (belly)

            % 4. Aerodynamic angles
            alpha = atan2(w, u);            
            
            % Kinematic sideslip: beta = asin(v / V_mag) with floating-point clamp
            if flow.v_mag > 1e-3
                v_ratio = v / flow.v_mag;
                v_ratio = min(max(v_ratio, -1.0), 1.0); % Clamp to strict [-1, 1] bounds
                beta = asin(v_ratio);
            else
                beta = 0.0;
            end
            
            % Angular rates P, Q, R (rad/s)
            p_rate = 0.0; q_rate = 0.0; r_rate = 0.0;
            if numel(X) >= 12
                p_rate = X(10);
                q_rate = X(11);
                r_rate = X(12);
            end

            % --- CLEANED KINEMATICS ---
            % No inverted hacks. Positive q = pitch up.
            alpha_dot = q_rate; % Bypass tan(beta) asymptote for planar stability
            
            disp(strcat('--- Alpha: ', num2str((alpha)), ' ---'));
            disp(strcat('--- Alpha_dot: ', num2str((alpha_dot)), ' ---'));
            disp(strcat('--- Beta: ', num2str((beta)), ' ---'));

            % Interpolate static, damping, and control coefficients at (Mach, Alpha)
            coeffs = obj.evaluateCoefficients(flow.mach, alpha);

            % Reference dimensions and Non-dimensional angular rate factors
            s_ref = obj.RefArea;
            d_ref = obj.RefLength;
            q_bar = flow.q_bar;
            v_inf = flow.v_mag;

            q_hat = (q_rate * d_ref) / (2.0 * v_inf);
            r_hat = (r_rate * d_ref) / (2.0 * v_inf);
            p_hat = (p_rate * d_ref) / (2.0 * v_inf);
            adot_hat = (alpha_dot * d_ref) / (2.0 * v_inf);

            % Total Force Coefficients (Body Axes)
            % Includes static baseline + rotation rates + lag (adot) + control deflections
            CA_total = coeffs.CA + (coeffs.CAQ * q_hat);
            CY_total = coeffs.CY + (coeffs.CYB * beta) + (coeffs.CYP * p_hat) + (coeffs.CYR * r_hat) + (coeffs.CYD * del_r);
            CN_total = coeffs.CN + (coeffs.CNQ * q_hat) + (coeffs.CNAD * adot_hat) + (coeffs.CND * del_q);

            qS = q_bar * s_ref;
            Fx_b = -CA_total * qS;
            Fy_b =  CY_total * qS;
            Fz_b = -CN_total * qS;

            F_aero = [Fx_b; Fy_b; Fz_b];

            % Total Moment Coefficients (Body Axes)
            % Standard non-hacked moment assignment
            Cl_total = coeffs.CLL + (coeffs.CLB * beta) + (coeffs.CLLP * p_hat) + (coeffs.CLLR * r_hat) + (coeffs.CLLD * del_p);
            Cm_total = coeffs.CM  + (coeffs.CMQ * q_hat)  + (coeffs.CMAD * adot_hat) + (coeffs.CMD * del_q);
            Cn_total = coeffs.CLN + (coeffs.CNB * beta) + (coeffs.CLNP * p_hat) + (coeffs.CLNR * r_hat) + (coeffs.CLND * del_r);

            % Vector order: [Roll (Mx); Pitch (My); Yaw (Mz)]
            M_aero = (q_bar * s_ref * d_ref) * [Cl_total; Cm_total; Cn_total];
        end

        function coeffs = evaluateCoefficients(obj, mach, alpha)
            mach_c  = min(max(mach,  min(obj.Mach)),  max(obj.Mach));
            alpha_c = min(max(alpha, min(obj.Alpha)), max(obj.Alpha));

            % Base Static
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
            coeffs.CAQ  = obj.interpField(obj.CAQ,  mach_c, alpha_c, 0.0);
            coeffs.CMAD = obj.interpField(obj.CMAD, mach_c, alpha_c, 0.0);
            coeffs.CNAD = obj.interpField(obj.CNAD, mach_c, alpha_c, 0.0);

            coeffs.CLNR = obj.interpField(obj.CLNR, mach_c, alpha_c, 0.0);
            coeffs.CYR  = obj.interpField(obj.CYR,  mach_c, alpha_c, 0.0);
            coeffs.CLLR = obj.interpField(obj.CLLR, mach_c, alpha_c, 0.0);
            coeffs.CLLP = obj.interpField(obj.CLLP, mach_c, alpha_c, 0.0);
            coeffs.CLNP = obj.interpField(obj.CLNP, mach_c, alpha_c, 0.0);
            coeffs.CYP  = obj.interpField(obj.CYP,  mach_c, alpha_c, 0.0);

            % Control Effectiveness Derivatives
            coeffs.CND  = obj.interpField(obj.CND,  mach_c, alpha_c, 0.0);
            coeffs.CMD  = obj.interpField(obj.CMD,  mach_c, alpha_c, 0.0);
            coeffs.CYD  = obj.interpField(obj.CYD,  mach_c, alpha_c, 0.0);
            coeffs.CLND = obj.interpField(obj.CLND, mach_c, alpha_c, 0.0);
            coeffs.CLLD = obj.interpField(obj.CLLD, mach_c, alpha_c, 0.0);
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
            side_dir = CoordinateFrame.lateralAxis(obj.Frame);
            if norm(side_dir) > 0
                side_dir = side_dir / norm(side_dir);
            end
            CY_total = coeffs.CY + (coeffs.CYB * beta);
            F_side = (flow.q_bar * CY_total * obj.RefArea) * side_dir;
        end
    end

    methods (Access = private)
        function val = interpField(obj, mat, mach_c, alpha_c, default_val)
            % Guard against entirely missing data (empty or all NaNs)
            if isempty(mat) || all(isnan(mat(:))) || all(mat(:) == 0)
                val = default_val;
            else
                val = interp2(obj.Mach, obj.Alpha, mat, mach_c, alpha_c, 'linear');
                % Guard against specific un-calculated flight regimes
                if isnan(val)
                    val = default_val;
                end
            end
        end
    end
end