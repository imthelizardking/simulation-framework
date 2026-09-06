%% File: Particle.m
classdef Particle < Munition
    % One fragment/sub-munition released from a Munition's burst. A
    % Particle IS-A Munition (per the framework's Base*+concrete pattern)
    % so it can be dropped straight into a Simulation-shaped loop: always
    % Dynamics3Dof (translational drag+gravity, no lift, no real attitude)
    % + AeroParticle, no Propulsion/Guidance/Controller/Sensors.
    properties
        Shape             (1,:) char = 'Sphere'
        Dimensions        struct     = struct()
        ReferenceAreaMode (1,:) char = 'Max'
    end

    methods
        function obj = Particle(name, shape, dims, cd, ref_mode, mass)
            if nargin < 1, name = 'Particle'; end
            obj = obj@Munition(name);
            if nargin > 1, obj.Shape = shape; end
            if nargin > 2, obj.Dimensions = dims; end
            if nargin < 4, cd = 1.0; end
            if nargin < 5, ref_mode = 'Max'; end
            obj.ReferenceAreaMode = ref_mode;
            if nargin > 5
                obj.MassProperties.Mass = mass;
            end

            obj.Dynamics   = Dynamics3Dof();
            obj.Aero       = AeroParticle(cd, obj.Shape, obj.Dimensions, obj.ReferenceAreaMode);
            obj.Propulsion = [];
            obj.Guidance   = [];
            obj.Controller = [];
            obj.Sensors    = {};
            obj.Charge     = [];
        end
    end
end
