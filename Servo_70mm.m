classdef Servo_70mm < Servo
    % SERVO_70MM Actuator model for 70mm rocket with 1st order lag and Hinge Moment limits
    
    properties
        max_deflection = deg2rad(15); % Max physical deflection [rad]
        max_rate = deg2rad(100);      % Max slew rate [rad/s]
        hm_limit = 7.0;               % Hinge moment stall limit [N-m]
    end
    
    methods
        function obj = Servo_70mm(time_constant)
            if nargin > 0
                obj.tau = time_constant;
            else
                obj.tau = 0.05; % Default 50ms time constant
            end
        end
        
        function delta_dot = compute_derivative(obj, delta_actual, delta_cmd, hm_actual)
            % Check if Hinge Moment exceeds motor torque limit (Stall condition)
            if abs(hm_actual) >= obj.hm_limit
                delta_dot = 0;
                return;
            end
            
            % Saturate command to max physical deflection
            delta_cmd_sat = max(min(delta_cmd, obj.max_deflection), -obj.max_deflection);
            
            % 1st order dynamics: dot = (cmd - actual) / tau
            delta_dot = (delta_cmd_sat - delta_actual) / obj.tau;
            
            % Rate Limiting
            delta_dot = max(min(delta_dot, obj.max_rate), -obj.max_rate);
        end
    end
end
