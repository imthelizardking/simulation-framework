classdef Environment < handle
    properties
        Gravity      (1,1) double = 9.81
        Density      (1,1) double = 1.225
        SpeedOfSound (1,1) double = 340.29 % m/s, ISA sea-level standard (constant - no altitude lapse modeled)
        BaseWind_NED (3,1) double = [0; 0; 0]
    end
    methods
        function obj = Environment(density, gravity, wind, speed_of_sound)
            if nargin > 0, obj.Density = density; end
            if nargin > 1, obj.Gravity = gravity; end
            if nargin > 2, obj.BaseWind_NED = wind; end
            if nargin > 3, obj.SpeedOfSound = speed_of_sound; end
        end
    end
end