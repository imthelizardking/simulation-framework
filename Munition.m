%% File: Munition.m
classdef Munition < handle
    properties
        Name           (1,:) char = 'Munition'
        Dynamics                  = []  % Holds instance of BaseDynamics subclass
        Aero                      = []  % Holds instance of BaseAero subclass
        Propulsion                = []  % Holds instance of BasePropulsion subclass
        Sensors        cell       = {}  % Cell array of BaseSensor subclasses
        Guidance                  = []  % Holds instance of BaseGuidance subclass
        Controller                = []  % Holds instance of BaseController subclass
        Charge                    = []  % Holds a ChargeZone (gun-fired munitions only; empty for rocket motors)
        MassProperties struct     = struct('Mass', 3.0, 'Inertia', eye(3))
        TwistRate      (1,1) double = 0.0 % rifling twist, calibers/turn; 0 = not applicable (fin-stabilized/no spin)
        LastGNCBus     struct     = struct()
    end

    methods
        function obj = Munition(name)
            if nargin > 0, obj.Name = name; end
        end

        function X0 = buildLaunchState(obj, launcher_pos, v0, elevation, azimuth)
            % Assembles the Dynamics-specific launch state, adding the
            % muzzle spin rate (from TwistRate + the round's bore
            % diameter) on top for a Dynamics3DofSpin munition - so
            % callers get one call site regardless of whether the munition
            % spins or not.
            if isa(obj.Dynamics, 'Dynamics3DofSpin') && obj.TwistRate > 0
                spin0 = Dynamics3DofSpin.spinRateFromTwist(v0, obj.TwistRate, obj.Aero.RefLength);
                X0 = obj.Dynamics.buildLaunchState(launcher_pos, v0, elevation, azimuth, obj.MassProperties.Mass, spin0);
            else
                X0 = obj.Dynamics.buildLaunchState(launcher_pos, v0, elevation, azimuth, obj.MassProperties.Mass);
            end
        end

        function updateGNC(obj, t, env, target)
            sensor_bus = struct();
            for i = 1:numel(obj.Sensors)
                s = obj.Sensors{i};
                sensor_bus.(s.Name) = s.step(t, obj.Dynamics.CurrentState, target, env);
            end
            
            nav_bus = obj.Dynamics.CurrentState;
            
            if ~isempty(obj.Guidance)
                guid_bus = obj.Guidance.step(t, nav_bus, target, sensor_bus);
            else
                guid_bus = struct('AccelCmd', 0.0);
            end
            
            if ~isempty(obj.Controller)
                ctrl_bus = obj.Controller.step(t, guid_bus, nav_bus, obj, env);
            else
                ctrl_bus = struct('FinDeflection', 0.0);
            end
            
            obj.LastGNCBus.Sensors    = sensor_bus;
            obj.LastGNCBus.Nav        = nav_bus;
            obj.LastGNCBus.Guidance   = guid_bus;
            obj.LastGNCBus.Controller = ctrl_bus;
        end
        
        function dX = getDerivatives(obj, t, X, env)
            obj.Dynamics.CurrentState = X;
            
            if ~isempty(obj.Propulsion)
                [F_prop, M_prop, m_dot] = obj.Propulsion.getThrust(t, env);
            else
                F_prop = [0;0;0]; M_prop = [0;0;0]; m_dot = 0;
            end
            
            if isfield(obj.LastGNCBus, 'Controller') && isfield(obj.LastGNCBus.Controller, 'FinDeflection')
                fin_deflection = obj.LastGNCBus.Controller.FinDeflection;
            else
                fin_deflection = 0.0;
            end
            
            if ~isempty(obj.Aero)
                [F_aero, M_aero] = obj.Aero.getForcesAndMoments(t, X, env, fin_deflection);
            else
                F_aero = [0;0;0]; M_aero = [0;0;0];
            end
            
            forces.Propulsion    = F_prop;
            forces.Aero          = F_aero;
            forces.FinDeflection = fin_deflection;
            moments.Propulsion   = M_prop;
            moments.Aero         = M_aero;
            
            dX = obj.Dynamics.computeDerivatives(t, X, forces, moments, obj.MassProperties, m_dot);
        end
    end
end