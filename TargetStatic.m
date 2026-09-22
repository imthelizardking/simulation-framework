%% File: TargetStatic.m
classdef TargetStatic < TargetBase
    methods
        function obj = TargetStatic(pos, reflectivity, frame)
            % frame (optional, default 'Z-Up') - see TargetBase's
            % constructor comment; pass 'FDR' to author pos directly in
            % forward/down/right instead of downrange/crossrange/altitude.
            if nargin < 1, pos = [0; 0; 0]; end
            if nargin < 3, frame = 'Z-Up'; end
            obj@TargetBase('TargetStatic', pos, [0; 0; 0], frame);
            if nargin > 1 && ~isempty(reflectivity), obj.LaserReflectivity = reflectivity; end
        end
        
        function step(~, ~, ~, ~)
            % Stationary target: position and velocity remain unchanged
        end
    end
end