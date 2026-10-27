classdef (Abstract) BaseDynamics < handle
    properties
        StateDimension (1,1) double
        InitialState   double
        CurrentState   double
    end

    properties (Constant)
        % Canonical full-state layout shared by the 6DoF/7DoF/MPMM blueprint
        % dynamics models: inertial position/velocity, Euler attitude, body
        % rates, a secondary spin state for dual-spin bodies, mass and
        % principal inertia. Each concrete model documents its own axis
        % convention for IDX_POS/IDX_VEL (DynamicsMPMM uses FDR - see that
        % class' comment); this layout only fixes the INDICES, not the
        % convention. Not every model integrates every slot (e.g. MPMM
        % holds IDX_EULER algebraically rather than integrating it), but all
        % of them share the same indices so generic code (Simulation,
        % telemetry, visualizers) never has to know which model is running.
        % Dynamics3Dof predates this and keeps its own 8-state
        % layout untouched.
        IDX_POS         = 1:3   % [x; y; z]        inertial position
        IDX_VEL         = 4:6   % [Vx; Vy; Vz]      inertial velocity
        IDX_EULER       = 7:9   % [phi; theta; psi] roll, pitch, yaw
        IDX_BODYRATE    = 10:12 % [p; q; r]         body angular rates
        IDX_SPIN2       = 13    % secondary/dual-spin state (rad/s)
        IDX_MASS        = 14    % vehicle mass
        IDX_INERTIA     = 15:17 % [Ixx; Iyy; Izz]   principal moments
        FULL_STATE_DIM  = 17
    end

    methods (Abstract)
        dX = computeDerivatives(obj, t, X, forces, moments, mass_prop, m_dot)
    end

    methods
        function mass = getMass(~, X)
            mass = X(BaseDynamics.IDX_MASS);
        end

        function X = setMassProperties(~, X, s)
            X(BaseDynamics.IDX_MASS) = s.Mass;
            X(BaseDynamics.IDX_INERTIA) = diag(s.Inertia);
        end
    end
end
