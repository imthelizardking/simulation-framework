%% File: TargetConstantVelocity.m
classdef TargetConstantVelocity < TargetBase
    methods
        function obj = TargetConstantVelocity(pos, vel)
            if nargin < 2, vel = [0; 0; 0]; end
            obj@TargetBase('TargetConstantVelocity', pos, vel);
        end

        function step(obj, ~, dt, ~)
            % Integrates motion every simulation step
            obj.Position = obj.Position + obj.Velocity * dt;
        end
    end
end
