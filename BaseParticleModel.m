%% File: BaseParticleModel.m
classdef (Abstract) BaseParticleModel < handle
    % Describes how a Munition's burst releases particles: where they sit
    % inside the body at burst (Positions, body-frame offsets from the CG)
    % and what shape/mass/drag they share. Positions is an N-by-3 matrix,
    % one row per particle: column 1 is the axial offset along the body's
    % forward/spin axis, columns 2-3 are the lateral offsets in the
    % cross-sectional plane - see Munition.spawnParticles for how this
    % body frame is built at burst time.
    properties
        Positions         (:,3) double = zeros(0, 3)
        Shape             (1,:) char   = 'Sphere' % 'Cylinder' | 'Sphere' | 'Cube' (alias 'Square')
        Dimensions        struct       = struct()
        CD                (1,1) double = 1.0
        ParticleMass      (1,1) double = 0.005
        ReferenceAreaMode (1,:) char   = 'Max'    % 'Min' | 'Max' - see AeroParticle.computeRefArea
    end

    methods
        function n = numParticles(obj)
            n = size(obj.Positions, 1);
        end
    end

    methods (Abstract)
        % N-by-3 body-frame velocity ADDED on top of the rigid-body
        % kinematics (forward + spin) every burst already applies -
        % zero rows mean "no separate ejection charge modeled".
        v_ejection = ejectionVelocities(obj)
    end
end
