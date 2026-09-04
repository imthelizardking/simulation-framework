classdef Environment < handle
    properties
        Gravity      (1,1) double = 9.81
        Density      (1,1) double = 1.225
        BaseWind_NED (3,1) double = [0; 0; 0]
    end
    methods
        function obj = Environment(density, gravity, wind)
            if nargin > 0, obj.Density = density; end
            if nargin > 1, obj.Gravity = gravity; end
            if nargin > 2, obj.BaseWind_NED = wind; end
        end
    end
end