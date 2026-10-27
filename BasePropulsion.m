classdef (Abstract) BasePropulsion < handle
    properties
        IsActive (1,1) logical = true
        FuelMass (1,1) double = 0.0
        MomentReference = [] % optional 3x1 body-datum position (m); [] uses legacy CGRef
    end
    methods (Abstract)
        [F_thrust, M_torque, m_dot] = getThrust(obj, t, env)
    end
end
