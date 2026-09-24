classdef ProportionalNav3D < BaseGuidance
    % True Proportional Navigation, full 3D vector form: a_cmd = N * Vc *
    % (omega_LOS x v_hat) - the standard vector generalization of the
    % classic scalar/planar PN law (Zarchan, "Tactical and Strategic
    % Missile Guidance"), unlike OptimalAngleFOVGuidance's own AccelCmd
    % (a single scalar meant for LongitudinalFinController's one fin
    % channel). The cross product with the missile's own velocity unit
    % vector guarantees the commanded acceleration is always perpendicular
    % to the flight path - no along-track component, which wouldn't be
    % achievable via aero lift anyway - and it nulls to zero exactly when
    % on a collision course (LOS rate -> 0), the defining PN behavior.
    properties
        N (1,1) double = 4.0 % navigation constant, typically 3-5
    end

    methods
        function obj = ProportionalNav3D()
            obj.MaxAccel = 40 * 9.81;
        end

        function guid_bus = step(obj, ~, nav_state, ~, sensor_bus)
            seeker = sensor_bus.Seeker3D;
            vel = nav_state(BaseDynamics.IDX_VEL);
            v_mag = norm(vel);

            % NOTE: deliberately NOT gating on seeker.IsBlind here anymore -
            % zeroing the command the instant look angle exceeds MaxFOV
            % created a vicious cycle with Autopilot6DoF's gentler gains
            % (Kp_alpha=1.0): a momentary lag pushes the nose past MaxFOV,
            % guidance cuts to zero right when correction is most needed,
            % the nose drifts further behind, and it never recovers.
            % IsBlind/LookAngle are still computed and reported by
            % Seeker3D for display (LiveEngagementPlotter's FOV lines) -
            % just not used to gate control here. Revisit if a scenario
            % genuinely needs "go ballistic while blind" behavior, with a
            % more generous MaxFOV so ordinary tracking lag doesn't trip it.
            if v_mag < 1e-3
                a_cmd = [0; 0; 0];
            else
                v_hat = vel / v_mag;
                a_cmd = obj.N * seeker.V_closure * cross(seeker.LOS_Rate_Vec, v_hat);
                mag = norm(a_cmd);
                if mag > obj.MaxAccel
                    a_cmd = a_cmd * (obj.MaxAccel / mag);
                end
            end

            guid_bus = struct('AccelCmd_Inertial', a_cmd, 'Range', seeker.Range);
        end
    end
end
