classdef Dynamics3DoF_PitchCoupled < BaseDynamics
    methods
        function obj = Dynamics3DoF_PitchCoupled()
            obj.StateDimension = 8; % [x; y; z; vx; vy; vz; pitch; mass]
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