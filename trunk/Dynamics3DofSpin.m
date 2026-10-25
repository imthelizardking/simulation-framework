classdef Dynamics3DofSpin < Dynamics3Dof
    % Dynamics3Dof plus a 9th state, spin rate p (rad/s) about the
    % velocity axis - the one thing Dynamics3Dof has no notion of. Pitch
    % is still handled by the same first-order pitch-follower heuristic as
    % the parent class (no real attitude/moment integration there); spin
    % is a genuine ODE state, integrated from the roll/spin-damping moment
    % moments.Aero(1). The spin
    % axis is assumed coincident with the velocity vector, valid while the
    % yaw of repose stays small - a full yaw-of-repose treatment lives in
    % DynamicsMPMM instead.
    methods
        function obj = Dynamics3DofSpin()
            obj = obj@Dynamics3Dof();
            obj.StateDimension = 9; % [x; y; z; vx; vy; vz; pitch; mass; spin]
        end

        function X0 = buildLaunchState(obj, launcher_pos, v0, elevation, azimuth, mass, spin0)
            if nargin < 7, spin0 = 0.0; end
            X0 = [buildLaunchState@Dynamics3Dof(obj, launcher_pos, v0, elevation, azimuth, mass); spin0];
        end

        function dX = computeDerivatives(obj, t, X, forces, moments, mass_prop, m_dot)
            dX8   = computeDerivatives@Dynamics3Dof(obj, t, X(1:8), forces, moments, mass_prop, m_dot);
            Ixx   = mass_prop.Inertia(1,1);
            dspin = moments.Aero(1) / Ixx;
            dX    = [dX8; dspin];
        end
    end

    methods (Static)
        function p0 = spinRateFromTwist(v0, twist_calibers, diameter)
            % Muzzle spin rate (rad/s) from a rifling twist rate expressed
            % in calibers per turn (one full revolution every
            % twist_calibers*diameter meters of bore travel) and the
            % shot's muzzle velocity.
            p0 = 2 * pi * v0 / (twist_calibers * diameter);
        end
    end
end
