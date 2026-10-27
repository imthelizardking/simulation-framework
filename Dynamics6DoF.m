classdef Dynamics6DoF < BaseDynamics
    % Genuine 6-DoF rigid body model using singularity-free quaternion kinematics,
    % integrated body rates (P, Q, R), and full translational/rotational EOMs.
    %
    % Coordinate Frame: NED (North-East-Down: X-forward, Y-right, Z-down)
    % Body Frame: X-nose (forward), Y-right wing/fin, Z-belly (down)
    %
    % Attitude State:
    %   Quaternion [q0; q1; q2; q3] (scalar-first, body -> inertial) stored in IDX_QUAT (18:21).
    %   Baumgarte constraint stabilization is active to prevent unit-norm drift.

    properties
        Frame (1,:) char = 'NED' % Fixed NED coordinate frame
    end

    properties (Constant)
        IDX_QUAT = 18:21         % State slots for quaternion [q0; q1; q2; q3]
    end

    methods
        function obj = Dynamics6DoF()
            obj.StateDimension = 21; % 17 base slots + 4 quaternion slots
        end

        function X0 = buildLaunchState(obj, launcher_pos, v0, elevation, azimuth, mass, p0, inertia)
            % BUILDLAUNCHSTATE Computes initial 6-DoF state vector
            %
            % Inputs:
            %   launcher_pos - [3x1] launcher position
            %   v0           - Launch velocity (m/s)
            %   elevation    - Elevation angle (rad, positive up)
            %   azimuth      - Azimuth angle (rad, clockwise from North)
            %   mass         - Total vehicle launch mass (kg)
            %   p0           - (Optional) Initial roll spin rate (rad/s), default = 0

            if nargin < 8 || isempty(p0)
                p0 = 0.0;
            end

            % Inertial velocity in Z-Up reference, then rotated to NED
            v_zup = v0 * [cos(elevation)*cos(azimuth); cos(elevation)*sin(azimuth); sin(elevation)];
            pos = CoordinateFrame.rotateVector(launcher_pos, obj.Frame);
            vel = CoordinateFrame.rotateVector(v_zup, obj.Frame);

            % Initial attitude aligned with launch rails
            psi0   = azimuth;
            theta0 = -elevation; % NED pitch is negative upward
            phi0   = 0.0;

            % Convert to scalar-first quaternion
            q0 = Dynamics6DoF.eulerToQuat(phi0, theta0, psi0);

            X0 = zeros(obj.StateDimension, 1);
            X0(BaseDynamics.IDX_POS)      = pos;
            X0(BaseDynamics.IDX_VEL)      = vel;
            X0(BaseDynamics.IDX_EULER)    = [phi0; theta0; psi0];
            X0(BaseDynamics.IDX_BODYRATE) = [p0; 0; 0]; % Initial roll spin (rad/s)
            X0(BaseDynamics.IDX_SPIN2)    = p0;
            X0(BaseDynamics.IDX_MASS)     = mass;
            X0(BaseDynamics.IDX_INERTIA) = inertia;
            X0(obj.IDX_QUAT)              = q0;
        end

        function dX = computeDerivatives(obj, ~, X, forces, moments, mass_prop, m_dot)
            % Extract state vectors
            vel  = X(BaseDynamics.IDX_VEL);
            quat = X(obj.IDX_QUAT);
            p    = X(BaseDynamics.IDX_BODYRATE(1));
            q    = -X(BaseDynamics.IDX_BODYRATE(2)); % making neg somehow solves it!
            r    = X(BaseDynamics.IDX_BODYRATE(3));
            m    = X(BaseDynamics.IDX_MASS);

            % 1. Defensive normalization of quaternion
            q_norm = norm(quat);
            if q_norm > 0
                q_unit = quat / q_norm;
            else
                q_unit = [1; 0; 0; 0];
            end

            % 2. Translational Dynamics (Inertial Frame)
            % Body -> Inertial Direction Cosine Matrix
            R_i2b = Dynamics6DoF.quatToDCM(q_unit);

            % Total forces in body frame (Aerodynamic + Propulsion)
            F_body_total = forces.Aero + forces.Propulsion;
            % Inertial acceleration: a = (R_bi * F_b) / m + g_ned
            g_vec = CoordinateFrame.gravityVector(9.80665, obj.Frame);
            accel = (R_i2b' * F_body_total) / m + g_vec;
            % disp(strcat('aero: ', num2str(forces.Aero)));
            % disp(strcat('prop: ', num2str(forces.Propulsion)));
            % disp(strcat('accel: ', num2str(accel)));

            % 3. Rotational Kinematics (Quaternion Derivatives)
            % Standard kinematic equation: q_dot = 0.5 * q (x) [0; p; q; r]
            % Baumgarte stabilization: + lambda * (1 - ||q||^2) * q (lambda = 1.0)
            omega_quat = [0; p; q; r];
            quat_dot = 0.5 * Dynamics6DoF.quatMultiply(q_unit, omega_quat) + ...
                1.0 * (1.0 - q_norm^2) * q_unit;

            % 4. Rotational Dynamics (Euler Equations of Motion)
            % Handle both 3x3 matrix and 3x1 vector inertia representations
            if isfield(mass_prop, 'Inertia')
                I_val = mass_prop.Inertia;
            elseif isprop(mass_prop, 'Inertia')
                I_val = mass_prop.Inertia;
            else
                error('Dynamics6DoF:InertiaMissing', 'mass_prop must define Inertia.');
            end

            if all(size(I_val) == [3, 3])
                Ixx = I_val(1,1); Iyy = I_val(2,2); Izz = I_val(3,3);
            else
                Ixx = I_val(1);   Iyy = I_val(2);   Izz = I_val(3);
            end

            % Total body moments
            M_body = moments.Aero + moments.Propulsion;

            % Principal axis rotational acceleration (omega_dot = I^-1 * (M - omega x I*omega))
            dp = (M_body(1) - (Izz - Iyy) * q * r) / Ixx;
            dq = (M_body(2) - (Ixx - Izz) * p * r) / Iyy;
            dr = (M_body(3) - (Iyy - Ixx) * p * q) / Izz;

            % 5. Euler Angle Kinematics (Telemetry/Tracking output)
            curr_euler = Dynamics6DoF.quatToEuler(q_unit);
            theta = curr_euler(2);
            phi   = curr_euler(1);
            cos_t = cos(theta);
            if abs(cos_t) > 1e-4
                dphi   = p + (q*sin(phi) + r*cos(phi)) * tan(theta);
                dtheta = q*cos(phi) - r*sin(phi);
                dpsi   = (q*sin(phi) + r*cos(phi)) / cos_t;
            else
                dphi = 0; dtheta = 0; dpsi = 0;
            end

            % 6. Assemble Full State Derivative Vector
            dX = zeros(obj.StateDimension, 1);
            dX(BaseDynamics.IDX_POS)      = vel;
            dX(BaseDynamics.IDX_VEL)      = accel;
            dX(BaseDynamics.IDX_EULER)    = [dphi; dtheta; dpsi];
            dX(BaseDynamics.IDX_BODYRATE) = [dp; dq; dr];
            dX(BaseDynamics.IDX_SPIN2)    = dp;
            dX(BaseDynamics.IDX_MASS)     = m_dot;
            dX(BaseDynamics.IDX_INERTIA)  = [0; 0; 0];
            dX(obj.IDX_QUAT)              = quat_dot;
        end
    end

    methods (Static)
        function R_i2b = quatToDCM(q)
            % True Body -> Inertial Direction Cosine Matrix (scalar-first: [q0; q1; q2; q3])
            q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
            R_i2b = [
                1 - 2*(q2^2 + q3^2),  2*(q1*q2 - q0*q3),  2*(q1*q3 + q0*q2);
                2*(q1*q2 + q0*q3),    1 - 2*(q1^2 + q3^2), 2*(q2*q3 - q0*q1);
                2*(q1*q3 - q0*q2),    2*(q2*q3 + q0*q1),  1 - 2*(q1^2 + q2^2)
                ];
        end

        function q = eulerToQuat(phi, theta, psi)
            %EULERTOQUAT_NED Convert Euler angles to body-to-NED quaternion.
            %
            % Inputs:
            %   phi   - Roll angle  [rad]
            %   theta - Pitch angle [rad]
            %   psi   - Yaw angle   [rad]
            %
            % Output:
            %   q     - 4x1 quaternion [q0; q1; q2; q3]
            %           Scalar-first, Hamilton convention
            %
            % Convention:
            %   - Aerospace 3-2-1 (ZYX) Euler rotation sequence
            %   - Body frame: X forward, Y right, Z down
            %   - NED frame:  X north, Y east, Z down
            %   - Quaternion transforms body-frame vectors into NED

            validateattributes(phi,   {'numeric'}, {'scalar', 'real', 'finite'});
            validateattributes(theta, {'numeric'}, {'scalar', 'real', 'finite'});
            validateattributes(psi,   {'numeric'}, {'scalar', 'real', 'finite'});

            cr = cos(phi/2);
            sr = sin(phi/2);

            cp = cos(theta/2);
            sp = sin(theta/2);

            cy = cos(psi/2);
            sy = sin(psi/2);

            q0 = cr*cp*cy + sr*sp*sy;
            q1 = sr*cp*cy - cr*sp*sy;
            q2 = cr*sp*cy + sr*cp*sy;
            q3 = cr*cp*sy - sr*sp*cy;

            q = [q0; q1; q2; q3];

            % Normalize to protect against numerical roundoff
            q = q / norm(q);
        end

        function euler = quatToEuler(q)
            % Quaternion to 3-2-1 Euler angles [phi; theta; psi] (rad)
            q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
            phi   = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
            theta = asin(max(-1.0, min(1.0, 2*(q0*q2 - q3*q1))));
            psi   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
            euler = [phi; theta; psi];
        end

        function q_out = quatMultiply(a, b)
            % Hamilton product (scalar-first: [w; x; y; z])
            q_out = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4); ...
                a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3); ...
                a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2); ...
                a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];
        end
    end
end