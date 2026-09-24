classdef Dynamics6DoF < BaseDynamics
    % Genuine 6-DoF rigid body: real attitude, integrated body rates, and
    % a real rotational EOM - unlike Dynamics3Dof/Dynamics3DofSpin
    % (point-mass, pitch as a first-order lag toward flight-path angle) or
    % AeroMPMM's yaw-of-repose (an ALGEBRAIC quasi-steady solve standing in
    % for real attitude tracking).
    %
    % ATTITUDE IS A QUATERNION (IDX_QUAT, this class' own constant - see
    % below), NOT the canonical IDX_EULER slot. An earlier version of this
    % class integrated the standard 3-2-1 Euler kinematics directly
    % (dphi = p + tan(theta)*(...), dpsi = (...)/cos(theta)) and hit a
    % real, confirmed problem: at sustained high pitch angles (a lofted
    % rocket trajectory, not just literal +-90 degrees), tan(theta)/
    % 1/cos(theta) amplify even a tiny numerically-seeded yaw rate into a
    % genuine, growing roll-angle artifact that feeds back into pitch -
    % gimbal lock's practical reach turned out to extend well before the
    % literal singularity. IDX_EULER (7:9) is kept in the state ONLY as a
    % one-time snapshot from buildLaunchState, for anything that expects
    % the canonical layout to be populated (matching BaseDynamics' own
    % "not every model integrates every slot" precedent - DynamicsMPMM
    % already holds IDX_EULER algebraically instead of integrating it, for
    % a different reason) - it is NEVER updated after launch (dX(IDX_EULER)
    % is always zero). Read IDX_QUAT (via quatToDCM) for the real, current
    % attitude - every class that needs it (AeroMissile6DoF, Autopilot6DoF,
    % Seeker3D, SALSeeker, GMLRSMidcourseGuidance) has been updated to do
    % this instead of reading IDX_EULER.
    %
    % Verified before replacing the Euler version: eulerToQuat/quatToDCM/
    % quatToEuler round-trip exactly against the original (already-
    % verified) Euler-based DCM to floating-point precision, and
    % quaternion vs. Euler kinematics integrated against an identical
    % time-varying p/q/r schedule agree to within 0.003 degrees after 10s
    % - this is a faithful, singularity-free replacement, not a different
    % model.
    %
    % Works natively in NED (X=forward, Y=right, Z=down) and only NED -
    % same "hardcode the one frame this class actually needs" choice
    % DynamicsMPMM made for FDR. Frame is still a real property so the
    % isprop(...,'Frame') fallbacks elsewhere in this framework
    % (Munition.spawnParticles, FireControlComputer.simulateFastBallistic)
    % read 'NED' correctly instead of guessing 'FDR' from StateDimension.
    %
    % IMPORTANT for any runner using this class: Simulation.run()
    % unconditionally propagates its OWN Frame onto Dynamics.Frame
    % (Simulation.Frame defaults to 'Z-Up') - exactly the same contract
    % every DynamicsMPMM runner already has to honor by setting
    % sim.Frame='FDR' (see runner_m107.m). Forgetting sim.Frame='NED' here
    % silently flips gravity to the WRONG vector (Z-Up's [0;0;-g] instead
    % of NED's [0;0;+g]) with no error - the trajectory will just look
    % wrong. Always set sim.Frame = 'NED' before sim.run().
    %
    % Body-axis convention: X = nose (forward), Y = right, Z = down (out
    % the belly) - standard aerospace body axes, matching NED's own sense.
    % Aero/Propulsion forces and moments (forces.Aero, moments.Aero, etc.)
    % are expected in THESE body axes, not inertial - see
    % computeDerivatives, which rotates F_total body->inertial once via
    % quatToDCM before integrating velocity/position.
    properties
        Frame (1,:) char = 'NED' % fixed - see this class' comment; not meant to be changed
    end

    properties (Constant)
        % This class' own state extension beyond BaseDynamics.
        % FULL_STATE_DIM (17) - a quaternion needs 4 slots, not the 3
        % IDX_EULER has room for. [q0;q1;q2;q3], scalar-first, body ->
        % inertial. Always re-normalize before use (quatToDCM does this
        % defensively) - RK4 integration lets the norm drift slightly
        % over many steps, and nothing in this class explicitly
        % renormalizes the STORED state (only its own local use of it),
        % so any other consumer must normalize too.
        IDX_QUAT = 18:21
    end

    methods
        function obj = Dynamics6DoF()
            obj.StateDimension = 21; % BaseDynamics.FULL_STATE_DIM (17) + 4 quaternion slots
        end

        function X0 = buildLaunchState(obj, launcher_pos, v0, elevation, azimuth, mass)
            % elevation/azimuth keep their familiar Z-Up sense (positive
            % elevation = aimed skyward), same as every other
            % buildLaunchState in this framework - built in Z-Up-native
            % trig first, then rotated to NED via the already-verified
            % CoordinateFrame.rotateVector.
            v_zup = v0 * [cos(elevation)*cos(azimuth); cos(elevation)*sin(azimuth); sin(elevation)];
            pos = CoordinateFrame.rotateVector(launcher_pos, obj.Frame);
            vel = CoordinateFrame.rotateVector(v_zup, obj.Frame);

            % Initial attitude: nose aligned with velocity (zero initial
            % angle of attack/sideslip - standard rail/tube-launch
            % assumption), wings level - derived from the already-rotated
            % NED velocity vector via the standard heading/elevation-from-
            % velocity formulas (see this class' history/verification
            % notes above for why, and Dynamics6DoF's earlier version for
            % the original derivation).
            psi0   = atan2(vel(2), vel(1));
            theta0 = atan2(-vel(3), hypot(vel(1), vel(2)));
            phi0   = 0.0;

            X0 = zeros(obj.StateDimension, 1);
            X0(BaseDynamics.IDX_POS)      = pos;
            X0(BaseDynamics.IDX_VEL)      = vel;
            X0(BaseDynamics.IDX_EULER)    = [phi0; theta0; psi0]; % one-time snapshot only - see class comment, NOT live
            X0(BaseDynamics.IDX_BODYRATE) = [0; 0; 0];
            X0(BaseDynamics.IDX_SPIN2)    = 0;
            X0(BaseDynamics.IDX_MASS)     = mass;
            X0(BaseDynamics.IDX_INERTIA)  = [0; 0; 0]; % unused - see computeDerivatives' own note
            X0(obj.IDX_QUAT)              = Dynamics6DoF.eulerToQuat(phi0, theta0, psi0);
        end

        function dX = computeDerivatives(obj, ~, X, forces, moments, mass_prop, m_dot)
            vel  = X(BaseDynamics.IDX_VEL);
            quat = X(obj.IDX_QUAT);
            p = X(BaseDynamics.IDX_BODYRATE(1));
            q = X(BaseDynamics.IDX_BODYRATE(2));
            r = X(BaseDynamics.IDX_BODYRATE(3));
            m = X(BaseDynamics.IDX_MASS);

            % Translational: body-axis forces rotated to inertial via the
            % quaternion DCM, then plain F=ma + gravity - the state's
            % velocity slot is INERTIAL, so (unlike the classic body-axis-
            % velocity formulation) there are no extra angular-rate
            % cross-coupling terms here: the DCM's own time-variation
            % already carries that coupling.
            R_bi = Dynamics6DoF.quatToDCM(quat);
            F_body_total = forces.Aero + forces.Propulsion;
            accel = (R_bi * F_body_total) / m + CoordinateFrame.gravityVector(9.81, obj.Frame);

            % Singularity-free quaternion kinematics (q_dot = 0.5*q*omega,
            % omega as a pure quaternion in BODY axes) - replaces the
            % Euler kinematics this class used to integrate directly. See
            % class comment for why.
            quat_dot = 0.5 * Dynamics6DoF.quatMultiply(quat, [0; p; q; r]);

            % Rotational EOM, principal-axis rigid body (I*omega_dot = M -
            % omega x I*omega, diagonal I). Inertia comes from mass_prop
            % (obj.MassProperties.Inertia), the same parameter
            % Dynamics3DofSpin already reads Ixx from - NOT from
            % X(IDX_INERTIA), which this class leaves as an unused
            % placeholder (see buildLaunchState).
            I = mass_prop.Inertia;
            Ixx = I(1,1); Iyy = I(2,2); Izz = I(3,3);
            M_body = moments.Aero + moments.Propulsion;
            dp = (M_body(1) - (Izz - Iyy) * q * r) / Ixx;
            dq = (M_body(2) - (Ixx - Izz) * p * r) / Iyy;
            dr = (M_body(3) - (Iyy - Ixx) * p * q) / Izz;

            dX = zeros(obj.StateDimension, 1);
            dX(BaseDynamics.IDX_POS)      = vel;
            dX(BaseDynamics.IDX_VEL)      = accel;
            dX(BaseDynamics.IDX_EULER)    = [0; 0; 0]; % frozen at its launch snapshot - not integrated, see class comment
            dX(BaseDynamics.IDX_BODYRATE) = [dp; dq; dr];
            dX(BaseDynamics.IDX_SPIN2)    = 0;
            dX(BaseDynamics.IDX_MASS)     = m_dot;
            dX(BaseDynamics.IDX_INERTIA)  = [0; 0; 0];
            dX(obj.IDX_QUAT)              = quat_dot;
        end
    end

    methods (Static)
        function R_bi = bodyToInertialDCM(phi, theta, psi)
            % Euler-angle DCM - kept for anything that only has Euler
            % angles on hand (e.g. a one-off report using IDX_EULER's
            % launch-time snapshot), NOT used by this class' own dynamics
            % anymore. See quatToDCM for the singularity-free version
            % computeDerivatives actually uses, and this class' header
            % comment for why the switch happened.
            Rx = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
            Ry = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
            Rz = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
            L_ib = Rx * Ry * Rz; % inertial -> body
            R_bi = L_ib.';       % body -> inertial (orthonormal: transpose = inverse)
        end

        function R_bi = quatToDCM(q)
            % Body -> inertial DCM from a scalar-first unit quaternion.
            % Normalizes defensively - see IDX_QUAT's own property
            % comment on why the stored quaternion isn't guaranteed exactly
            % unit norm. Verified to match bodyToInertialDCM exactly
            % (to floating-point precision) via eulerToQuat - see this
            % class' header comment.
            q = q / norm(q);
            q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
            R_bi = [1-2*(q2^2+q3^2), 2*(q1*q2-q0*q3),   2*(q1*q3+q0*q2);
                    2*(q1*q2+q0*q3), 1-2*(q1^2+q3^2),   2*(q2*q3-q0*q1);
                    2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1),   1-2*(q1^2+q2^2)];
        end

        function q = eulerToQuat(phi, theta, psi)
            % Standard 3-2-1 (yaw-pitch-roll) Euler-to-quaternion
            % conversion, matching bodyToInertialDCM/quatToDCM's shared
            % convention - verified by round-tripping through both DCM
            % formulas and comparing (see class header comment).
            cr = cos(phi/2);   sr = sin(phi/2);
            cp = cos(theta/2); sp = sin(theta/2);
            cy = cos(psi/2);   sy = sin(psi/2);
            q = [cr*cp*cy + sr*sp*sy; ...
                 sr*cp*cy - cr*sp*sy; ...
                 cr*sp*cy + sr*cp*sy; ...
                 cr*cp*sy - sr*sp*cy];
        end

        function euler = quatToEuler(q)
            % Inverse of eulerToQuat - for reporting/telemetry only (e.g.
            % printing a human-readable attitude). Has the same well-known
            % (but harmless, since nothing integrates through it) reading
            % ambiguity exactly at theta = +-90 degrees any Euler-angle
            % readout has - the quaternion itself, and everything this
            % class actually computes from it, stays perfectly well-
            % defined regardless.
            q = q / norm(q);
            q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
            phi   = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
            theta = asin(max(-1, min(1, 2*(q0*q2 - q3*q1))));
            psi   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
            euler = [phi; theta; psi];
        end

        function q_out = quatMultiply(a, b)
            % Hamilton product, scalar-first convention - matches
            % eulerToQuat/quatToDCM's shared convention throughout.
            q_out = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4); ...
                     a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3); ...
                     a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2); ...
                     a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];
        end
    end
end
