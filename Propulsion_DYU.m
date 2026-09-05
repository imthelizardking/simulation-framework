classdef Propulsion_DYU < handle
    % Very basic base-bleed (Dip Yanma Unitesi) model: a fixed fractional
    % reduction of CD0 while the unit burns, full drag afterward. Real
    % base-bleed units taper off gradually as the gas-generator charge
    % depletes; this is a step function, not a burn-rate simulation - good
    % enough to capture "reduced drag for the boost/glide phase, normal
    % drag after," not meant to reproduce a real gas-generation profile.
    % Note: despite the Propulsion_ naming, this does not extend
    % BasePropulsion - it modifies drag, not thrust, so it doesn't
    % implement getThrust and isn't assigned to Munition.Propulsion.
    properties
        BurnDuration         (1,1) double = 30.0  % s
        DragReductionFraction (1,1) double = 0.25 % fraction of CD0 removed while active
    end

    methods
        function obj = Propulsion_DYU(burn_duration, drag_reduction_fraction)
            if nargin > 0, obj.BurnDuration = burn_duration; end
            if nargin > 1, obj.DragReductionFraction = drag_reduction_fraction; end
        end

        function factor = dragMultiplier(obj, t)
            if t <= obj.BurnDuration
                factor = 1.0 - obj.DragReductionFraction;
            else
                factor = 1.0;
            end
        end
    end
end
