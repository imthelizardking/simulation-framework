classdef (Abstract) BaseController < handle
    properties
        MaxDeflection (1,1) double = deg2rad(20)
    end
    methods (Abstract)
        ctrl_bus = step(obj, t, guid_bus, nav_state, veh_cfg, env)
    end
end