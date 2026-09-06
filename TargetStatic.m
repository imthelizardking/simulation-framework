%% File: TargetStatic.m
classdef TargetStatic < TargetBase
    methods
        function obj = TargetStatic(pos, reflectivity)
            if nargin < 1, pos = [0; 0; 0]; end
            obj@TargetBase('TargetStatic', pos, [0; 0; 0]);
            if nargin > 1, obj.LaserReflectivity = reflectivity; end
        end
        
        function step(~, ~, ~, ~)
            % Stationary target: position and velocity remain unchanged
        end
    end
end