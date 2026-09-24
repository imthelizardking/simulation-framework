classdef Imu < BaseSensor
    % Strapdown IMU: gyro (body rates) + accelerometer (specific force),
    % each corrupted by a slowly-drifting bias (a random walk - "bias
    % instability", the real mechanism behind long-term IMU drift) plus
    % white measurement noise - not just flat Gaussian noise on the truth,
    % which would never actually accumulate into the kind of drift a real
    % IMU produces. This class only produces the raw corrupted
    % measurements; integrating them into a drifting position/attitude
    % estimate (where the drift actually becomes visible/consequential) is
    % done by the guidance that consumes this sensor (see
    % GMLRSMidcourseGuidance's dead-reckoning INS mechanization) - the
    % same "sensor measures, something else fuses" split GnssSensor/this
    % class have from GMLRSMidcourseGuidance.
    %
    % Accelerometer physics: an accelerometer measures SPECIFIC FORCE
    % (non-gravitational force per unit mass - a free-falling body reads
    % zero), not total acceleration. BaseSensor.step has no access to
    % Aero/Propulsion forces directly (unlike Controller.step, which gets
    % the whole Munition) - rather than extending that interface, this
    % class numerically differentiates TRUTH velocity between consecutive
    % calls and subtracts the known gravity vector, which is exactly
    % what a real accelerometer's own output is physically equivalent to.
    % Requires calling every physics step (not at some slower rate) for
    % the differencing to stay accurate - unlike GnssSensor, which is
    % deliberately slow.
    properties
        GyroBiasInstability  (1,1) double = deg2rad(0.5)  % rad/s per sqrt(s) - random-walk rate
        GyroNoiseStd         (1,1) double = deg2rad(0.05) % rad/s, white noise, 1-sigma
        AccelBiasInstability (1,1) double = 0.005          % m/s^2 per sqrt(s) - random-walk rate
        AccelNoiseStd        (1,1) double = 0.02            % m/s^2, white noise, 1-sigma
    end

    properties (Access = private)
        GyroBias  (3,1) double = [0;0;0]
        AccelBias (3,1) double = [0;0;0]
        LastVel   double = []
        LastT     (1,1) double = -Inf
    end

    methods
        function obj = Imu()
            obj.Name = 'IMU';
        end

        function meas = step(obj, t, truth_state, ~, ~)
            omega_true = truth_state(BaseDynamics.IDX_BODYRATE);
            vel_true   = truth_state(BaseDynamics.IDX_VEL);

            if isinf(obj.LastT)
                dt = 0.0;
            else
                dt = t - obj.LastT;
            end

            % Random-walk bias update, scaled by sqrt(dt) so the process
            % statistics don't change with step size (standard discrete-
            % time Wiener-process integration).
            if dt > 0
                obj.GyroBias  = obj.GyroBias  + obj.GyroBiasInstability  * sqrt(dt) * randn(3,1);
                obj.AccelBias = obj.AccelBias + obj.AccelBiasInstability * sqrt(dt) * randn(3,1);
            end

            gyro_meas = omega_true + obj.GyroBias + obj.GyroNoiseStd * randn(3,1);

            if isempty(obj.LastVel) || dt <= 0
                specific_force_body = [0; 0; 0]; % first call - no velocity history yet to differentiate
            else
                accel_true_inertial = (vel_true - obj.LastVel) / dt;
                g_vec = CoordinateFrame.gravityVector(9.81, 'NED');
                specific_force_inertial = accel_true_inertial - g_vec;
                % Quaternion (Dynamics6DoF.IDX_QUAT), not the frozen
                % IDX_EULER launch-time snapshot - see that class' header
                % comment.
                R_bi = Dynamics6DoF.quatToDCM(truth_state(Dynamics6DoF.IDX_QUAT));
                specific_force_body = R_bi.' * specific_force_inertial;
            end
            accel_meas = specific_force_body + obj.AccelBias + obj.AccelNoiseStd * randn(3,1);

            obj.LastVel = vel_true;
            obj.LastT   = t;

            meas = struct('GyroRate', gyro_meas, 'SpecificForce', accel_meas);
        end
    end
end
