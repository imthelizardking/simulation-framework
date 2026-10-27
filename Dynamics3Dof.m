classdef Dynamics3Dof < BaseDynamics
    properties
        Frame (1,:) char = 'Z-Up' % 'Z-Up' or 'NED' - see CoordinateFrame; must match the paired Aero
    end
    methods
        function mass = getMass(~, X)
            mass = X(8);
        end

        function X = setMassProperties(~, X, s)
            X(8) = s.Mass;
        end

        function obj = Dynamics3Dof()
            obj.StateDimension = 8; % [x; y; z; vx; vy; vz; pitch; mass]
        end

        function X0 = buildLaunchState(obj, launcher_pos, v0, elevation, azimuth, mass)
            % Assembles [pos; vel; pitch; mass] from firing angles and
            % launch speed, so callers pass elevation/azimuth directly
            % instead of hand-deriving vx/vy/vz via trig each time.
            % elevation/azimuth always keep their familiar "Z-Up" meaning
            % (positive elevation = aimed skyward) regardless of
            % obj.Frame - the whole resulting state is rotated to the
            % active frame as the very last step (see CoordinateFrame),
            % so a caller (e.g. FireControlComputer's solvers) never has
            % to think about which frame is active.
            v_x = v0 * cos(elevation) * cos(azimuth);
            v_y = v0 * cos(elevation) * sin(azimuth);
            v_z = v0 * sin(elevation);
            pos = CoordinateFrame.rotateVector(launcher_pos, obj.Frame);
            vel = CoordinateFrame.rotateVector([v_x; v_y; v_z], obj.Frame);
            pitch = CoordinateFrame.rotateAngle(elevation, obj.Frame);
            X0 = [pos; vel; pitch; mass];
        end

        function dX = computeDerivatives(obj, ~, X, forces, ~, ~, m_dot)
            vel   = X(4:6);
            pitch = X(7);
            m     = X(8);

            % Sum translational forces: Aero (Lift + Drag) + Thrust + Gravity
            F_total = forces.Aero + forces.Propulsion + m * CoordinateFrame.gravityVector(9.81, obj.Frame);
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
