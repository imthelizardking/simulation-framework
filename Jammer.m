classdef Jammer < handle
    % Spatial GNSS jamming zone: full denial (Valid=false) within
    % EffectiveRadius of Position - see GnssSensor.Jammer. Deliberately
    % minimal (no power/frequency modeling) - a real GNSS jammer's effect
    % is overwhelmingly "receiver loses lock inside some radius," not a
    % gradual degradation, so a hard radius cutoff is the honest first
    % approximation, same spirit as ChargeZone/Battery's plain data-holder
    % classes elsewhere in this framework.
    properties
        Position        (3,1) double = [0; 0; 0]
        EffectiveRadius (1,1) double = 5000.0 % m
    end

    methods
        function obj = Jammer(position, effective_radius)
            if nargin > 0, obj.Position = position; end
            if nargin > 1, obj.EffectiveRadius = effective_radius; end
        end
    end
end
