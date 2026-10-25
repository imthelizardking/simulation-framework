%% File: TargetConstantVelocity.m
classdef TargetConstantVelocity < BaseTarget
    methods
        function obj = TargetConstantVelocity(pos, vel, frame)
            % frame (optional, default 'Z-Up') - see BaseTarget's
            % constructor comment; pass 'FDR' to author pos/vel directly
            % in forward/down/right instead of downrange/crossrange/altitude.
            if nargin < 2, vel = [0; 0; 0]; end
            if nargin < 3, frame = 'Z-Up'; end
            obj@BaseTarget('TargetConstantVelocity', pos, vel, frame);
        end

        function step(obj, ~, dt, ~)
            % Integrates motion every simulation step
            obj.Position = obj.Position + obj.Velocity * dt;
        end
    end
end
