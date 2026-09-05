classdef Dynamics3Dof < BaseDynamics
    methods
        function obj = Dynamics3Dof()
            obj.StateDimension = 8; % [x; y; z; vx; vy; vz; pitch; mass]
        end

        function X0 = buildLaunchState(~, launcher_pos, v0, elevation, azimuth, mass)
            % Assembles [pos; vel; pitch; mass] from firing angles and
            % launch speed, so callers pass elevation/azimuth directly
            % instead of hand-deriving vx/vy/vz via trig each time.
            v_x = v0 * cos(elevation) * cos(azimuth);
            v_y = v0 * cos(elevation) * sin(azimuth);
            v_z = v0 * sin(elevation);
            X0 = [launcher_pos; v_x; v_y; v_z; elevation; mass];
        end

        function dX = computeDerivatives(~, ~, X, forces, ~, ~, m_dot)
            vel   = X(4:6);
            pitch = X(7);
            m     = X(8);
            
            % Sum translational forces: Aero (Lift + Drag) + Thrust + Gravity (Z-Up)
            F_total = forces.Aero + forces.Propulsion + [0; 0; -m * 9.81];
            accel   = F_total / m;
            
            % Longitudinal Pitch Follower Dynamics
            v_mag = norm(vel);
            if v_mag > 1e-3
                gamma = atan2(vel(3), vel(1));
                target_pitch = gamma + forces.FinDeflection * 2.0;
                dpitch = (target_pitch - pitch) * 10.0;
            else
                dpitch = 0;
            end
            
            dX = [vel; accel; dpitch; m_dot];
        end
    end
end