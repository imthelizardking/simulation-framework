classdef SolidMotor < BasePropulsion
    properties
        ThrustForce  (1,1) double = 1200.0
        BurnDuration (1,1) double = 2.0
    end
    
    methods
        function obj = SolidMotor(thrust, burn_time)
            if nargin > 0, obj.ThrustForce = thrust; end
            if nargin > 1, obj.BurnDuration = burn_time; end
        end
        
        function [F_thrust, M_torque, m_dot] = getThrust(obj, t, ~)
            if t <= obj.BurnDuration
                F_thrust = [obj.ThrustForce; 0; 0];
            else
                F_thrust = [0; 0; 0];
            end
            M_torque = [0; 0; 0];
            m_dot = 0.0;
        end
    end
end