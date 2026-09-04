classdef (Abstract) BaseDynamics < handle
    properties
        StateDimension (1,1) double
        InitialState   double
        CurrentState   double
    end
    methods (Abstract)
        dX = computeDerivatives(obj, t, X, forces, moments, mass_prop, m_dot)
    end
end