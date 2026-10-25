classdef AeroMissile6DoF < BaseAero
    % Body-axis 6-DoF missile aero: real alpha/beta computed from the
    % vehicle's TRUE attitude vs. its velocity vector (via Dynamics6DoF's
    % own DCM), unlike AeroMPMM's yaw-of-repose (an algebraic stand-in for
    % attitude a point-mass model doesn't track) or Aero6DoF's planar
    % alpha-only, wind-axis CD/CL, force-only model (paired with
    % Dynamics3Dof, which has no real rotational state to feed moments
    % into). Designed to be fed by Missile DATCOM-derived coefficients
    % (CN/CY/CA/Cm/Cn/Cl-style), supplied via the standard MunitionData
    % aero.<phase>.<Coeff> struct exactly like every other Aero class -
    % see MunitionFactory for how a preset wires this up. Requires an
    % axisymmetric, cruciform-finned airframe: pitch-plane coefficients
    % (alpha, q, pitch-fin deflection) and yaw-plane coefficients (beta,
    % r, yaw-fin deflection) are kept as SEPARATE fields rather than one
    % being derived from the other, so real DATCOM numbers decide whether
    % they're actually equal, not a hardcoded symmetry assumption - but
    % the equations themselves assume no pitch/yaw cross-coupling beyond
    % what's already in the rotational EOM.
    %
    % Body axes: X=forward(nose), Y=right, Z=down - matches Dynamics6DoF.
    % Sign conventions below (CA opposes +X, CN acts along -Z for
    % positive alpha, CY acts along +Y for positive beta) follow the
    % standard Missile DATCOM/aeroballistics convention, but have only
    % been checked against known-good LIMITS (zero-alpha/beta symmetry,
    % restoring-moment sign for a statically-stable body) - not against
    % real tabulated numbers, since none exist yet (damping/control
    % derivatives default to zero - see their own property comments).
    % Re-verify every sign here against your actual DATCOM output once
    % you have it, the same way the 25mm RefArea/IForm data errors were
    % only caught by comparing against real numbers.
    properties
        % Static coefficients - function of alpha (pitch plane) or beta
        % (yaw plane), each itself Mach-dependent via BaseAero.evalCoeff
        % (scalar, Table, Polynomial, or PiecewisePolynomial - see that
        % method). Linear-in-alpha/beta buildup, same structural pattern
        % as AeroMPMM's CLA/CMA slopes - add a cubic term here the same
        % way (a new *_alpha3/*_beta3 property plus one extra term in
        % evaluateCoefficients) if your data needs it; not added
        % speculatively.
        CN_alpha (1,1) double = 2.0   % normal-force slope, pitch plane, 1/rad
        CY_beta  (1,1) double = 2.0   % side-force slope, yaw plane, 1/rad (pitch/yaw symmetric for a true axisymmetric body - kept separate, see class comment)
        CA0      (1,1) double = 0.3   % zero-alpha/beta axial force (drag)
        CA_alpha2(1,1) double = 0     % axial-force rise with alpha^2 (drag-due-to-lift)
        Cm_alpha (1,1) double = -2.0  % pitching-moment slope (NEGATIVE = statically stable: restoring moment opposes alpha)
        Cn_beta  (1,1) double = -2.0  % yawing-moment slope (NEGATIVE = directionally stable, weathercock)
        Cl_beta  (1,1) double = 0     % rolling moment due to sideslip - ~0 for a true axisymmetric body; kept for honesty, not assumed zero
        %% gemini adddition:
        CN_delta (1,1) double = 0 % Normal force per unit pitch-fin deflection
        CY_delta (1,1) double = 0 % Side force per unit yaw-fin deflection
        %% gemini adddition:
        % Damping (rate) derivatives - need a Missile DATCOM $DAMP run to
        % populate for real; 0 (default) = no rate damping at all, which
        % will tend to leave pitch/yaw oscillations underdamped. See
        % MunitionFactory for how these read from data/<Name>.mat's
        % aero.phase.* the same way the static coefficients above do.
        Cm_q (1,1) double = 0   % pitch damping
        Cn_r (1,1) double = 0   % yaw damping
        Cl_p (1,1) double = 0   % roll damping

        % Control effectiveness - need a Missile DATCOM $CONTROL run to
        % populate for real; 0 (default) = fin deflections produce no
        % moment at all (an autopilot built against this would have
        % nothing to actuate with until these are set).
        Cm_delta (1,1) double = 0   % pitching moment per unit pitch-fin deflection
        Cn_delta (1,1) double = 0   % yawing moment per unit yaw-fin deflection
        Cl_delta (1,1) double = 0   % rolling moment per unit roll/differential-fin deflection
    end

    methods
        function [F_aero, M_aero] = getForcesAndMoments(obj, ~, X, env, fin_deflections)
            p = X(BaseDynamics.IDX_BODYRATE(1));
            q = X(BaseDynamics.IDX_BODYRATE(2));
            r = X(BaseDynamics.IDX_BODYRATE(3));

            % True alpha/beta from actual attitude vs. velocity - not an
            % algebraic yaw-of-repose stand-in, since Dynamics6DoF really
            % does track attitude. Reads the quaternion (Dynamics6DoF.
            % IDX_QUAT), NOT the frozen IDX_EULER launch-time snapshot -
            % see that class' header comment on why IDX_EULER is no longer
            % live. R_bi is body->inertial and orthonormal, so its
            % transpose (not an explicit inverse) rotates inertial
            % velocity into body axes.
            R_bi = Dynamics6DoF.quatToDCM(X(Dynamics6DoF.IDX_QUAT));
            v_body = R_bi.' * X(BaseDynamics.IDX_VEL);
            V = norm(v_body);
            if V < 1e-3
                F_aero = [0; 0; 0];
                M_aero = [0; 0; 0];
                return;
            end
            alpha = atan2(v_body(3), v_body(1));
            beta  = asin(max(-1, min(1, v_body(2) / V)));

            altitude = CoordinateFrame.altitude(X, 'NED');
            ro    = env.getDensity(altitude);
            mach  = V / env.getSpeedOfSound(altitude);
            q_bar = 0.5 * ro * V^2;

            [d_pitch, d_yaw, d_roll] = obj.resolveFinDeflections(fin_deflections);
            coeffs = obj.evaluateCoefficients(mach);

            F_aero = obj.forceEquation(coeffs, alpha, beta, q_bar);
            M_aero = obj.momentEquation(coeffs, alpha, beta, p, q, r, V, q_bar, d_pitch, d_yaw, d_roll);
        end

        function coeffs = evaluateCoefficients(obj, mach)
            coeffs.CN_alpha  = BaseAero.evalCoeff(obj.CN_alpha,  mach);
            coeffs.CY_beta   = BaseAero.evalCoeff(obj.CY_beta,   mach);
            coeffs.CA0       = BaseAero.evalCoeff(obj.CA0,       mach);
            coeffs.CA_alpha2 = BaseAero.evalCoeff(obj.CA_alpha2, mach);
            coeffs.Cm_alpha  = BaseAero.evalCoeff(obj.Cm_alpha,  mach);
            coeffs.Cn_beta   = BaseAero.evalCoeff(obj.Cn_beta,   mach);
            coeffs.Cl_beta   = BaseAero.evalCoeff(obj.Cl_beta,   mach);
            coeffs.Cm_q      = BaseAero.evalCoeff(obj.Cm_q,      mach);
            coeffs.Cn_r      = BaseAero.evalCoeff(obj.Cn_r,      mach);
            coeffs.Cl_p      = BaseAero.evalCoeff(obj.Cl_p,      mach);
            coeffs.Cm_delta  = BaseAero.evalCoeff(obj.Cm_delta,  mach);
            coeffs.Cn_delta  = BaseAero.evalCoeff(obj.Cn_delta,  mach);
            coeffs.Cl_delta  = BaseAero.evalCoeff(obj.Cl_delta,  mach);
        end

        function F_body = forceEquation(obj, coeffs, alpha, beta, q_bar)
            CA = coeffs.CA0 + coeffs.CA_alpha2 * alpha^2;
            CN = coeffs.CN_alpha * alpha;
            %% gemini addition:
            CY = coeffs.CY_beta * beta;
            %CN = coeffs.CN_alpha * alpha + coeffs.CN_delta * d_pitch;CY = coeffs.CY_beta * beta   + coeffs.CY_delta * d_yaw;
            %% gemini addition            
            % Standard body-axis force convention: axial force opposes
            % +X (drag), normal force acts along -Z for positive alpha
            % (nose-up relative wind -> "lift"-like force), side force
            % along +Y for positive beta - see class comment.
            F_body = q_bar * obj.RefArea * [-CA; CY; -CN];
        end

        function M_body = momentEquation(obj, coeffs, alpha, beta, p, q, r, V, q_bar, d_pitch, d_yaw, d_roll)
            % Standard nondimensional rate terms (rate * RefLength / 2V) -
            % zero when V~0 is already excluded by getForcesAndMoments'
            % own early return above.
            p_hat = p * obj.RefLength / (2 * V);
            q_hat = q * obj.RefLength / (2 * V);
            r_hat = r * obj.RefLength / (2 * V);

            Cl = coeffs.Cl_beta * beta  + coeffs.Cl_p * p_hat + coeffs.Cl_delta * d_roll;
            Cm = coeffs.Cm_alpha * alpha + coeffs.Cm_q * q_hat + coeffs.Cm_delta * d_pitch;
            Cn = coeffs.Cn_beta * beta  + coeffs.Cn_r * r_hat + coeffs.Cn_delta * d_yaw;

            M_body = q_bar * obj.RefArea * obj.RefLength * [Cl; Cm; Cn];
        end
    end

    methods (Static, Access = private)
        function [d_pitch, d_yaw, d_roll] = resolveFinDeflections(fin_deflections)
            % Multi-channel fin deflection, one struct field per axis -
            % the abstract getForcesAndMoments signature already names
            % this parameter `fin_deflections` (plural), anticipating
            % exactly this; every other concrete Aero class just never
            % needed more than one channel. Falls back to all-zero for
            % the plain scalar 0.0 every other class' convention passes
            % when no Controller is attached yet (see
            % Munition.getDerivatives) - not a valid 3-channel input, but
            % a sensible "no control input" default while this vehicle
            % has no autopilot wired up.
            if isstruct(fin_deflections)
                d_pitch = fin_deflections.Pitch;
                d_yaw   = fin_deflections.Yaw;
                d_roll  = fin_deflections.Roll;
            else
                d_pitch = 0.0;
                d_yaw   = 0.0;
                d_roll  = 0.0;
            end
        end
    end
end
