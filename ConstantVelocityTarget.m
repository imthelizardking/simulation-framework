%% File: ConstantVelocityTarget.m
classdef ConstantVelocityTarget < BaseTarget
    methods
        function obj = ConstantVelocityTarget(pos, vel)
            if nargin < 2, vel = [0; 0; 0]; end
            obj@BaseTarget('ConstantVelocityTarget', pos, vel);
        end

        function step(obj, ~, dt, ~)
            % Integrates motion every simulation step
            obj.Position = obj.Position + obj.Velocity * dt;
        end
    end
end
