classdef ChargeZone < handle
    % One specific propelling-charge configuration for a gun-fired
    % munition: MuzzleVelocity/MuzzleVelocityStdDev are specific to the
    % (munition, charge) pair they're attached to via MunitionFactory -
    % matching how real firing tables are tabulated per projectile, since
    % the same physical charge gives different velocities in different
    % shells.
    properties
        Name                 (1,:) char   = 'Charge'
        Propellant           (1,:) char   = ''
        MuzzleVelocity       (1,1) double = 0.0
        MuzzleVelocityStdDev (1,1) double = 0.0
    end

    methods
        function obj = ChargeZone(name, propellant, v0, stddev)
            if nargin > 0, obj.Name = name; end
            if nargin > 1, obj.Propellant = propellant; end
            if nargin > 2, obj.MuzzleVelocity = v0; end
            if nargin > 3, obj.MuzzleVelocityStdDev = stddev; end
        end
    end
end
