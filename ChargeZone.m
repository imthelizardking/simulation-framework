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

        % STANAG 4355 MPMM fitting/form factors (see AeroMPMM.IForm/FL/
        % QD/QM) - defined per propelling charge, not per munition, since
        % they characterize that charge's own pressure/burn regime.
        % Default 1.0 (identity gain, no correction) when a charge has no
        % override data - see ChargeFactory for how these get populated
        % from a data file.
        IForm (1,1) double = 1.0
        FL    (1,1) double = 1.0
        QD    (1,1) double = 1.0
        QM    (1,1) double = 1.0
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
