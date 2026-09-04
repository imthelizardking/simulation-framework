%% File: Simulation.m
classdef Simulation < handle
    properties
        Munition                  = []  % Munition container OR raw BaseDynamics object
        Target                    = []  % BaseTarget subclass OR struct
        Environment               = []  % Environment object
        Designator                = []  % BaseDesignator subclass (optional)
        GroundStation             = []  % GroundStation object (optional)
        Visualizer                = []  % BaseVisualizer subclass (optional)
        TimeSpan  (1,2) double    = [0 25]
        Dt        (1,1) double    = 0.001
        Telemetry       struct    = struct()
    end
    
    methods
        function obj = Simulation(munition, target, env, timespan, dt)
            if nargin > 0, obj.Munition = munition; end
            if nargin > 1, obj.Target = target; end
            if nargin > 2, obj.Environment = env; end
            if nargin > 3, obj.TimeSpan = timespan; end
            if nargin > 4, obj.Dt = dt; end
        end
        
        function tel = run(obj)
            dt = obj.Dt;
            time_array = obj.TimeSpan(1):dt:obj.TimeSpan(2);
            N_steps = length(time_array);
            
            % 1. Extract initial state X
            if isprop(obj.Munition, 'Dynamics') && ~isempty(obj.Munition.Dynamics)
                X = obj.Munition.Dynamics.InitialState;
                obj.Munition.Dynamics.CurrentState = X;
            elseif isprop(obj.Munition, 'InitialState')
                X = obj.Munition.InitialState;
                obj.Munition.CurrentState = X;
            else
                error('Simulation:Munition', 'Munition has no valid InitialState.');
            end
            
            % 2. Preallocate Telemetry
            tel.time       = zeros(N_steps, 1);
            tel.pos        = zeros(N_steps, 3);
            tel.vel        = zeros(N_steps, 3);
            tel.pitch      = zeros(N_steps, 1);
            tel.target_pos = zeros(N_steps, 3);
            tel.accel_cmd  = zeros(N_steps, 1);
            tel.range      = zeros(N_steps, 1);
            
            % 3. Resolve Target Bus
            tgt_bus = obj.resolveTargetBus();
            
            % 4. Initialize Visualizer if attached
            if ~isempty(obj.Visualizer)
                bounds = [-100, max(X(1), tgt_bus.pos(1)) + 100, 0, max(X(3), tgt_bus.pos(3)) + 300];
                obj.Visualizer.init(X, tgt_bus, bounds);
            end
            
            step_count = 0;
            for k = 1:N_steps
                t = time_array(k);
                step_count = step_count + 1;
                
                % Step Target Dynamics
                if isobject(obj.Target) && ismethod(obj.Target, 'step')
                    obj.Target.step(t, dt, X);
                end
                tgt_bus = obj.resolveTargetBus();
                
                % Step External Laser Designator (if present)
                if ~isempty(obj.Designator) && ismethod(obj.Designator, 'step')
                    obj.Designator.step(t, dt, tgt_bus);
                    des_bus = obj.Designator.getDesignatorBus();
                else
                    des_bus = struct('IsDesignating', false, 'PRF_Code', 0);
                end
                
                % Step Ground Station / Offboard Radar (if present)
                if ~isempty(obj.GroundStation) && ismethod(obj.GroundStation, 'step')
                    uplink_data = obj.GroundStation.step(t, dt, X(1:3), tgt_bus.pos);
                else
                    uplink_data = struct('Valid', false);
                end
                
                % Step GNC ONLY if Munition supports it
                world_truth = struct(...
                    'Target',       tgt_bus, ...
                    'Environment',  obj.Environment, ...
                    'Designator',   des_bus, ...
                    'GroundUplink', uplink_data);
                
                if ismethod(obj.Munition, 'updateGNC')
                    obj.Munition.updateGNC(t, obj.Environment, tgt_bus);
                end
                
                % Telemetry Logging
                tel.time(k)          = t;
                tel.pos(k, :)        = X(1:3)';
                tel.vel(k, :)        = X(4:6)';
                if length(X) >= 7, tel.pitch(k) = X(7); end
                tel.target_pos(k, :) = tgt_bus.pos';
                
                if isprop(obj.Munition, 'LastGNCBus') && isfield(obj.Munition.LastGNCBus, 'Guidance') && isfield(obj.Munition.LastGNCBus.Guidance, 'AccelCmd')
                    tel.accel_cmd(k) = obj.Munition.LastGNCBus.Guidance.AccelCmd;
                end
                tel.range(k) = norm(tgt_bus.pos - X(1:3));
                
                % Step Visualizer
                if ~isempty(obj.Visualizer)
                    gnc_bus = struct();
                    if isprop(obj.Munition, 'LastGNCBus'), gnc_bus = obj.Munition.LastGNCBus; end
                    obj.Visualizer.step(t, X, tgt_bus, gnc_bus);
                end
                
                % Check Termination Criteria
                if obj.checkTermination(X, tgt_bus.pos)
                    break;
                end
                
                % Integrate via RK4
                X = obj.rk4Step(t, X, dt);
            end
            
            % Truncate telemetry
            fields = fieldnames(tel);
            for i = 1:numel(fields)
                tel.(fields{i}) = tel.(fields{i})(1:step_count, :);
            end
            obj.Telemetry = tel;
        end
        
        function is_term = checkTermination(~, X, target_pos)
            r_tgt = norm(target_pos - X(1:3));
            is_term = (r_tgt < 3.0) || (X(3) < 0 && norm(X(4:6)) > 0) || ((X(1) - target_pos(1)) > 50.0);
        end
        
        function X_next = rk4Step(obj, t, X, dt)
            k1 = obj.evalDerivatives(t, X);
            k2 = obj.evalDerivatives(t + 0.5*dt, X + 0.5*dt*k1);
            k3 = obj.evalDerivatives(t + 0.5*dt, X + 0.5*dt*k2);
            k4 = obj.evalDerivatives(t + dt, X + dt*k3);
            X_next = X + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);
        end
        
        function dX = evalDerivatives(obj, t, X)
            if ismethod(obj.Munition, 'getDerivatives')
                dX = obj.Munition.getDerivatives(t, X, obj.Environment);
            elseif ismethod(obj.Munition, 'computeDerivatives')
                % Direct BaseDynamics object bypass
                forces.Aero          = [0; 0; 0];
                forces.Propulsion    = [0; 0; 0];
                forces.FinDeflection = 0.0;
                moments.Aero         = [0; 0; 0];
                moments.Propulsion   = [0; 0; 0];
                mass_prop.Mass       = X(end);
                m_dot                = 0.0;
                dX = obj.Munition.computeDerivatives(t, X, forces, moments, mass_prop, m_dot);
            else
                error('Simulation:Munition', 'Munition must implement getDerivatives or computeDerivatives.');
            end
        end
        
        function tgt_bus = resolveTargetBus(obj)
            if isobject(obj.Target) && ismethod(obj.Target, 'getTargetBus')
                tgt_bus = obj.Target.getTargetBus();
            elseif isstruct(obj.Target)
                tgt_bus = obj.Target;
            else
                tgt_bus = struct('pos', [0;0;0], 'vel', [0;0;0], 'accel', [0;0;0], ...
                                 'LaserReflectivity', 0.3, 'ThermalSignature', 300, 'RCS', 1.0);
            end
        end
    end
end