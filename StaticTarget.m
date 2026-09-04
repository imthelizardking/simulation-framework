%% File: StaticTarget.m
classdef StaticTarget < BaseTarget
    methods
        function obj = StaticTarget(pos, reflectivity)
            if nargin < 1, pos = [0; 0; 0]; end
            obj@BaseTarget('StaticTarget', pos, [0; 0; 0]);
            if nargin > 1, obj.LaserReflectivity = reflectivity; end
        end
        
        function step(~, ~, ~, ~)
            % Stationary target: position and velocity remain unchanged
        end
    end
end