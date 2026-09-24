classdef Seeker3D < BaseSensor
    % Full 3D seeker for a genuine 6-DoF vehicle - unlike SeekerLOAL (which
    % hardcodes a single-plane LOS via atan2(r_vec(3),r_vec(1)) and reads
    % truth_state(7) as "pitch", both specific to Dynamics3Dof's 8-state
    % planar layout), this computes a real 3D line-of-sight rotation-rate
    % VECTOR (omega_LOS = cross(r,v_rel)/|r|^2 - the standard proportional-
    % navigation quantity), so engagement geometry with lateral offset/
    % motion works correctly, not just same-vertical-plane intercepts.
    %
    % FOV/lock tracking mirrors SeekerLOAL's own T_SeekerOn/MaxFOV pattern,
    % but the look angle is the TRUE 3D angle between the vehicle's actual
    % nose direction (from Dynamics6DoF's own DCM - real attitude, not a
    % planar pitch proxy) and the line of sight, via dot(nose,LOS) - exact
    % for any 3D engagement geometry, including lateral offset/motion
    % SeekerLOAL's planar angle difference could never represent correctly.
    properties
        MaxFOV     (1,1) double = deg2rad(30)
        T_SeekerOn (1,1) double = 0.0 % 0 = active from launch; set >0 for a LOAL-style uncage delay
    end

    methods
        function obj = Seeker3D(max_fov, t_seeker_on)
            obj.Name = 'Seeker3D';
            if nargin > 0, obj.MaxFOV = max_fov; end
            if nargin > 1, obj.T_SeekerOn = t_seeker_on; end
        end

        function meas = step(obj, t, truth_state, target, ~)
            pos = truth_state(BaseDynamics.IDX_POS);
            vel = truth_state(BaseDynamics.IDX_VEL);

            r_vec = target.pos - pos;
            dist  = norm(r_vec);
            v_rel = target.vel - vel;

            omega_los = cross(r_vec, v_rel) / max(dist^2, 1e-3);
            V_c = -dot(v_rel, r_vec / max(dist, 1e-3));

            % Quaternion (Dynamics6DoF.IDX_QUAT), not the frozen IDX_EULER
            % launch-time snapshot - see that class' header comment.
            R_bi     = Dynamics6DoF.quatToDCM(truth_state(Dynamics6DoF.IDX_QUAT));
            nose_hat = R_bi * [1; 0; 0];
            los_hat  = r_vec / max(dist, 1e-3);
            look_angle = acos(max(-1, min(1, dot(nose_hat, los_hat))));

            if t < obj.T_SeekerOn
                is_blind = false; is_locked = false; % virtual/INS phase, same convention as SeekerLOAL
            elseif look_angle > obj.MaxFOV
                is_blind = true;  is_locked = false;
            else
                is_blind = false; is_locked = true;
            end

            meas = struct('Range', dist, 'LOS_Rate_Vec', omega_los, 'V_closure', max(V_c, 10.0), ...
                'NoseDir', nose_hat, 'MaxFOV', obj.MaxFOV, 'LookAngle', look_angle, ...
                'IsBlind', is_blind, 'IsLocked', is_locked);
        end
    end
end
