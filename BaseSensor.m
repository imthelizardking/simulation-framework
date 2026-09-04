classdef (Abstract) BaseSensor < handle
    properties
        Name       (1,:) char = 'Sensor'
        UpdateRate (1,1) double = 1000 % Hz
    end
    methods (Abstract)
        meas = step(obj, t, truth_state, target, env)
    end
end