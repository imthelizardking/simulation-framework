classdef Seeker_SAL < BaseSensor
    % Semi-Active Laser seeker: unlike Seeker_3D (always "sees" the target
    % geometrically, gated only by its own FOV), this seeker detects
    % NOTHING unless a LaserDesignator is actively illuminating the target
    % with line-of-sight (see that class) AND its own PRF_Code matches
    % (an illustrative anti-spoof/deconfliction check - a real SAL seeker
    % is coded to one designator's pulse pattern so it doesn't home on a
    % stray laser). Holds a DIRECT reference to a LaserDesignator object
    % (set by the runner) - see LaserDesignator's own comment on why this
    % is the chosen pattern over Simulation's unwired Designator/des_bus
    % hook.
    %
    % Field names (IsBlind/IsLocked/LookAngle/NoseDir/MaxFOV) deliberately
    % match Seeker_3D's own measurement shape so Visualizer_LiveEngagement's
    % existing 3D-seeker FOV-line drawing branch can recognize either
    % sensor by name without a third near-duplicate branch.
    properties
        Designator = []                     % LaserDesignator reference - required; no detection at all without one
        PRF_Code   (1,1) double = 1688       % must match Designator.PRF_Code
        MaxFOV     (1,1) double = deg2rad(20)
        ActivationRange (1,1) double = 4000.0 % m - real terminal seekers uncage late in flight, not from launch
    end

    methods
        function obj = Seeker_SAL(designator)
            obj.Name = 'SAL';
            if nargin > 0, obj.Designator = designator; end
        end

        function meas = step(obj, ~, truth_state, target, ~)
            pos = truth_state(BaseDynamics.IDX_POS);
            vel = truth_state(BaseDynamics.IDX_VEL);

            r_vec = target.pos - pos;
            dist  = norm(r_vec);
            v_rel = target.vel - vel;
            los_hat = r_vec / max(dist, 1e-3);

            % Quaternion (Dynamics6DoF.IDX_QUAT), not the frozen IDX_EULER
            % launch-time snapshot - see that class' header comment.
            R_bi = Dynamics6DoF.quatToDCM(truth_state(Dynamics6DoF.IDX_QUAT));
            nose_hat = R_bi * [1; 0; 0];
            look_angle = acos(max(-1, min(1, dot(nose_hat, los_hat))));

            is_locked = ~isempty(obj.Designator) && dist <= obj.ActivationRange && look_angle <= obj.MaxFOV ...
                && obj.Designator.illuminates(target.pos) && obj.Designator.PRF_Code == obj.PRF_Code;

            if is_locked
                omega_los = cross(r_vec, v_rel) / max(dist^2, 1e-3);
                V_c = -dot(v_rel, los_hat);
                meas = struct('IsLocked', true, 'IsBlind', false, 'Range', dist, ...
                    'LOS_Rate_Vec', omega_los, 'V_closure', max(V_c, 10.0), ...
                    'LookAngle', look_angle, 'NoseDir', nose_hat, 'MaxFOV', obj.MaxFOV);
            else
                meas = struct('IsLocked', false, 'IsBlind', true, 'Range', dist, ...
                    'LOS_Rate_Vec', [0;0;0], 'V_closure', 0.0, ...
                    'LookAngle', look_angle, 'NoseDir', nose_hat, 'MaxFOV', obj.MaxFOV);
            end
        end
    end
end
