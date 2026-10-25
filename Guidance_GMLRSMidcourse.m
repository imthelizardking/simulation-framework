classdef Guidance_GMLRSMidcourse < BaseGuidance
    % Two-phase guidance for a GNSS/IMU midcourse + SAL terminal munition:
    %
    % MIDCOURSE (no SAL lock yet): a real GMLRS-class round has no way to
    % track a moving target before its terminal seeker uncages - it flies
    % toward a FIXED aimpoint (the target's known/estimated position at
    % launch, i.e. the fire mission's grid coordinate), using its own
    % onboard dead-reckoning INS estimate, not ground truth. That INS
    % estimate is maintained HERE (not in SensorIMU/GnssSensor - see those
    % classes' own comments on the "sensor measures, guidance fuses"
    % split): every step, IMU gyro/accel propagate the estimate forward
    % (a real strapdown mechanization - attitude from gyro, velocity from
    % accel rotated by the ESTIMATED attitude plus analytic gravity, since
    % an accelerometer never senses gravity itself), and any fresh GNSS
    % fix resets it (a hard reset, not a Kalman blend - consistent with
    % this framework not using Kalman filters anywhere else). Attitude
    % error from gyro bias genuinely leaks into position/velocity drift
    % here, exactly like a real INS - this is deliberate, not a bug.
    %
    % The internal attitude estimate is a QUATERNION (QuatEst), not Euler
    % angles - this class' mechanization originally integrated Euler
    % kinematics directly and would hit the same gimbal-lock-adjacent
    % problem Dynamics6DoF did at sustained high pitch (a lofted midcourse
    % trajectory spends real time up there) - see that class' header
    % comment for the full explanation and verification. Uses the exact
    % same Dynamics6DoF.quatMultiply/quatToDCM/eulerToQuat helpers.
    %
    % TERMINAL (SAL locked): a fixed aimpoint can't track a MOVING target
    % - once Seeker_SAL reports a lock, this switches entirely to true 3D
    % proportional navigation driven directly by the seeker's own LOS-rate
    % measurement (same formula as Guidance_ProportionalNav3D, reusing nav_state's
    % true velocity for the PN geometry the same way that class does - see
    % its own comment on why nav_state is ground truth throughout this
    % framework and that's fine once a seeker is providing the actual
    % bearing measurement).
    properties
        AimPoint     (3,1) double = [0; 0; 0] % NED - fixed midcourse target point, set by the runner
        N_Midcourse  (1,1) double = 2.0        % gentler than terminal - just general course-following toward a wide-open aimpoint
        N_Terminal   (1,1) double = 4.0
    end

    properties (Access = private)
        Initialized (1,1) logical = false
        PosEst   (3,1) double = [0;0;0]
        VelEst   (3,1) double = [0;0;0]
        QuatEst  (4,1) double = [1;0;0;0]
        LastT    (1,1) double = -Inf
    end

    methods
        function obj = Guidance_GMLRSMidcourse()
            obj.MaxAccel = 25 * 9.81; % illustrative - a large guided rocket, less agile than a small interceptor
        end

        function guid_bus = step(obj, t, nav_state, ~, sensor_bus)
            if ~obj.Initialized
                % INS alignment at launch - a real system surveys/aligns
                % against a known launcher position and initial attitude
                % before firing, so seeding from truth exactly once here
                % is realistic, not a shortcut.
                obj.PosEst  = nav_state(BaseDynamics.IDX_POS);
                obj.VelEst  = nav_state(BaseDynamics.IDX_VEL);
                obj.QuatEst = nav_state(Dynamics6DoF.IDX_QUAT);
                obj.LastT = t;
                obj.Initialized = true;
            end
            dt = t - obj.LastT;
            obj.LastT = t;

            imu = sensor_bus.IMU;
            if dt > 0
                p = imu.GyroRate(1); q = imu.GyroRate(2); r = imu.GyroRate(3);
                quat_dot = 0.5 * Dynamics6DoF.quatMultiply(obj.QuatEst, [0; p; q; r]);
                obj.QuatEst = obj.QuatEst + quat_dot * dt;
                obj.QuatEst = obj.QuatEst / norm(obj.QuatEst);

                R_bi_est = Dynamics6DoF.quatToDCM(obj.QuatEst);
                g_vec = CoordinateFrame.gravityVector(9.81, 'NED');
                accel_est_inertial = R_bi_est * imu.SpecificForce + g_vec;
                obj.VelEst = obj.VelEst + accel_est_inertial * dt;
                obj.PosEst = obj.PosEst + obj.VelEst * dt;
            end

            gnss = sensor_bus.GNSS;
            if gnss.Valid
                obj.PosEst = gnss.Position;
                obj.VelEst = gnss.Velocity;
            end

            sal = sensor_bus.SAL;
            if sal.IsLocked
                vel_true = nav_state(BaseDynamics.IDX_VEL);
                v_mag = norm(vel_true);
                if v_mag < 1e-3
                    a_cmd = [0; 0; 0];
                else
                    v_hat = vel_true / v_mag;
                    a_cmd = obj.N_Terminal * sal.V_closure * cross(sal.LOS_Rate_Vec, v_hat);
                    a_cmd = obj.saturate(a_cmd);
                end
                phase = 'Terminal';
            else
                r_vec = obj.AimPoint - obj.PosEst;
                dist  = norm(r_vec);
                v_mag = norm(obj.VelEst);
                if v_mag < 1e-3 || dist < 1e-3
                    a_cmd = [0; 0; 0];
                else
                    v_hat   = obj.VelEst / v_mag;
                    los_hat = r_vec / dist;
                    % PN toward a STATIONARY aimpoint (target velocity 0),
                    % same formula Guidance_ProportionalNav3D uses for a moving
                    % one - the fixed point just makes v_rel = -VelEst.
                    omega_los = cross(r_vec, -obj.VelEst) / max(dist^2, 1e-3);
                    Vc = dot(obj.VelEst, los_hat);
                    a_cmd = obj.N_Midcourse * max(Vc, 10.0) * cross(omega_los, v_hat);
                    a_cmd = obj.saturate(a_cmd);
                end
                phase = 'Midcourse';
            end

            guid_bus = struct('AccelCmd_Inertial', a_cmd, 'Phase', phase, ...
                'EstPos', obj.PosEst, 'EstVel', obj.VelEst);
        end
    end

    methods (Access = private)
        function a_cmd = saturate(obj, a_cmd)
            mag = norm(a_cmd);
            if mag > obj.MaxAccel
                a_cmd = a_cmd * (obj.MaxAccel / mag);
            end
        end
    end
end
