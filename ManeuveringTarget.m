%% File: ManeuveringTarget.m
classdef ManeuveringTarget < BaseTarget
    properties
        ManeuverType (1,:) char   = 'ConstantVelocity'
        ManeuverG    (1,1) double = 0.0
    end
    
    methods
        function obj = ManeuveringTarget(pos, vel, maneuver_type)
            if nargin < 2, vel = [0; 0; 0]; end
            if nargin < 3, maneuver_type = 'ConstantVelocity'; end
            obj@BaseTarget('ManeuveringTarget', pos, vel);
            obj.ManeuverType = maneuver_type;
        end
        
        function step(obj, ~, dt, ~)
            % Integrates motion every simulation step
            obj.Position = obj.Position + obj.Velocity * dt;
        end
    end
end