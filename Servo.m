classdef Servo < handle
    % SERVO Base class for actuator modeling
    
    properties
        tau % Time constant for first order lag, or damping/nat_freq for 2nd order
    end
    
    methods
        function obj = Servo()
        end
        
        function delta_dot = compute_derivative(obj, delta_actual, delta_cmd, hm_actual)
            % Base method to be overridden
            delta_dot = 0;
        end
    end
end
