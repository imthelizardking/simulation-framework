classdef (Abstract) BaseGuidance < handle
    properties
        Gain     (1,1) double = 3.5
        MaxAccel (1,1) double = 40 * 9.81
    end
    methods (Abstract)
        guid_bus = step(obj, t, nav_state, target, sensor_bus)
    end
end