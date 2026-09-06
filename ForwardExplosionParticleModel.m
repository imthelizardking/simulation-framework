%% File: ForwardExplosionParticleModel.m
classdef ForwardExplosionParticleModel < BaseParticleModel
    % The one particle-release model implemented so far: particles are
    % arranged cylindrically inside the munition body (Positions) and, at
    % burst, inherit ONLY the parent's forward velocity and its spin-induced
    % side velocity (rigid-body kinematics v = v_cg + spin x r, applied in
    % Munition.spawnParticles) - no separate explosive ejection speed. That
    % matches "forward explosion": the burst is a structural separation
    % that releases the particles into the parent's own motion, rather than
    % a charge that sprays them outward on top of it.
    methods
        function obj = ForwardExplosionParticleModel(positions, shape, dims, cd, mass, ref_mode)
            if nargin > 0, obj.Positions = positions; end
            if nargin > 1, obj.Shape = shape; end
            if nargin > 2, obj.Dimensions = dims; end
            if nargin > 3, obj.CD = cd; end
            if nargin > 4, obj.ParticleMass = mass; end
            if nargin > 5 && ~isempty(ref_mode), obj.ReferenceAreaMode = ref_mode; end
        end

        function v_ejection = ejectionVelocities(obj)
            v_ejection = zeros(obj.numParticles(), 3);
        end
    end
end
