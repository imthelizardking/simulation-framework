classdef (Abstract) BasePropulsion < handle
    properties
        IsActive (1,1) logical = true
        FuelMass (1,1) double = 0.0
    end
    methods (Abstract)
        [F_thrust, M_torque, m_dot] = getThrust(obj, t, env)
    end
end