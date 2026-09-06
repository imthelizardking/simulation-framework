%% File: Simulation.m
classdef Simulation < handle
    properties
        Munition                  = []  % Munition container OR raw BaseDynamics object
        Target                    = []  % TargetBase subclass OR struct
        Environment               = []  % Environment object
        Designator                = []  % BaseDesignator subclass (optional)
        GroundStation             = []  % GroundStation object (optional)
        Visualizer                = []  % BaseVisualizer subclass (optional)
        Integrator                = RK4FixedStep()  % BaseIntegrator subclass
        TimeSpan  (1,2) double    = [0 25]
        Dt        (1,1) double    = 0.001
        Telemetry       struct    = struct()
        % Termination criteria; a field set to [] is disabled. Default is
        % ground-impact only (MinAltitude) - the one criterion that applies
        % to every scenario, guided or not. Target-relative criteria
        % (ImpactDistance, MaxDownrange) are opt-in: enable them per-
        % scenario for intercept/engagement runs where a "miss" needs its
        % own stopping condition.
        Termination struct = struct(...
            'MaxTime',        [], ...   % s;  stop once t >= MaxTime
            'ImpactDistance', [], ...   % m;  stop once within this of target
            'MinAltitude',    0, ...    % m;  stop on descending through this
            'MaxAltitude',    [], ...   % m;  stop on climbing through this
            'MaxDownrange',   [], ...   % m;  stop this far past target in X
            'PassedTarget',   false)    % stop once range-to-target starts increasing (closest approach is behind it) - see evalTerminationCriteria
        TerminationReason (1,:) char = ''
        LastState          double       = []  % X at the moment run() terminated - needed by triggerFragmentation
        % Fragmentation hand-off (only used if Munition.BurstTime is set -
        % see checkTermination/triggerFragmentation): once the main flight
        % terminates with reason 'Burst', run() hands off to a
        % ParticleSwarmSimulation built from Munition.spawnParticles.
        ParticleVisualizer     = []  % BaseVisualizer subclass for the post-burst swarm phase
        ParticleTermination    struct = struct()  % overrides ParticleSwarmSimulation's default Termination if non-empty
        ParticleTimeSpan  (1,2) double = [0 15]
        ParticleDt              double = []  % [] = reuse obj.Dt
        ParticleSwarm           = []  % holds the resulting ParticleSwarmSimulation after triggerFragmentation
    end
    
    methods
        function obj = Simulation(munition, target, env, timespan, dt)
            % Dock every figure this MATLAB session creates from here on
            % (Visualizer live views, post-run plots, etc.) - set once here
            % so no runner script has to repeat it.
            set(0, 'DefaultFigureWindowStyle', 'docked');
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

                % Re-authoritative CurrentState before GNC reads it: the
                % integrator's derivative evaluations (RK4's intermediate
                % k1..k4 sub-steps) set Dynamics.CurrentState as a side
                % effect of every call, so by this point in the loop it
                % holds whichever sub-step X was last evaluated at during
                % the PREVIOUS iteration's integration - not the properly
                % combined X this iteration actually starts from. Guidance/
                % sensors must see the real current state, not that stale
                % intermediate value.
                if isprop(obj.Munition, 'Dynamics') && ~isempty(obj.Munition.Dynamics)
                    obj.Munition.Dynamics.CurrentState = X;
                elseif isprop(obj.Munition, 'CurrentState')
                    obj.Munition.CurrentState = X;
                end

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
                [is_term, term_reason] = obj.checkTermination(t, X, tgt_bus.pos, tgt_bus.vel);
                if is_term
                    obj.TerminationReason = term_reason;
                    obj.LastState = X;
                    break;
                end

                % Integrate
                X = obj.Integrator.step(@(tt, XX) obj.evalDerivatives(tt, XX), t, X, dt);
            end

            % Truncate telemetry
            fields = fieldnames(tel);
            for i = 1:numel(fields)
                tel.(fields{i}) = tel.(fields{i})(1:step_count, :);
            end
            obj.Telemetry = tel;

            if strcmp(obj.TerminationReason, 'Burst')
                obj.triggerFragmentation();
            end
        end

        function [is_term, reason] = checkTermination(obj, t, X, target_pos, target_vel)
            % Burst preempts the standard criteria: it isn't a hit/miss
            % read on the parent body, it's a hand-off to the fragment
            % swarm (see triggerFragmentation), so it must win any
            % coincidental tie with e.g. MinAltitude at the same step.
            if isa(obj.Munition, 'Munition') && ~isempty(obj.Munition.BurstTime) && t >= obj.Munition.BurstTime
                is_term = true; reason = 'Burst'; return;
            end
            [is_term, reason] = Simulation.evalTerminationCriteria(obj.Termination, t, X, target_pos, target_vel);
        end

        function triggerFragmentation(obj)
            % Hands off from the just-terminated single-body flight to a
            % lockstep multi-particle swarm phase (see spawnParticles for
            % the burst kinematics, ParticleSwarmSimulation for the
            % per-particle lockstep loop).
            particles = obj.Munition.spawnParticles(obj.LastState, obj.Environment);

            dt = obj.Dt;
            if ~isempty(obj.ParticleDt), dt = obj.ParticleDt; end

            swarm = ParticleSwarmSimulation(particles, obj.Target, obj.Environment, obj.ParticleTimeSpan, dt);
            if ~isempty(fieldnames(obj.ParticleTermination))
                swarm.Termination = obj.ParticleTermination;
            end
            swarm.Visualizer = obj.ParticleVisualizer;
            swarm.run();
            obj.ParticleSwarm = swarm;
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

    methods (Static)
        function [is_term, reason] = evalTerminationCriteria(T, t, X, target_pos, target_vel)
            % Shared by Simulation.checkTermination and
            % ParticleSwarmSimulation (each fragment is checked against the
            % same criteria shape - ImpactDistance = "meets the target",
            % MinAltitude/MaxDownrange/PassedTarget = "loses its chance").
            % Evaluates each enabled criterion in T (any field left []/false
            % is skipped) and stops on the first one that trips.
            % MinAltitude/MaxAltitude read X(3) directly, so they assume
            % whichever vertical-axis convention the active Dynamics model
            % uses (Z-up here).
            is_term = false;
            reason  = '';

            if ~isempty(T.MaxTime) && t >= T.MaxTime
                is_term = true; reason = 'MaxTime'; return;
            end
            if ~isempty(T.ImpactDistance) && norm(target_pos - X(1:3)) < T.ImpactDistance
                is_term = true; reason = 'ImpactDistance'; return;
            end
            if ~isempty(T.MinAltitude) && X(3) < T.MinAltitude && norm(X(4:6)) > 0
                is_term = true; reason = 'MinAltitude'; return;
            end
            if ~isempty(T.MaxAltitude) && X(3) > T.MaxAltitude
                is_term = true; reason = 'MaxAltitude'; return;
            end
            if isfield(T, 'PassedTarget') && T.PassedTarget
                % d(range)/dt = (los . v_rel) / range, where los points
                % from the body to the target and v_rel is the target's
                % velocity relative to the body. >= 0 means range is no
                % longer shrinking - closest approach is already behind
                % it, so any hit is already reflected in ImpactDistance
                % above and it will only diverge from here.
                los = target_pos - X(1:3);
                range_now = norm(los);
                if range_now > 1e-6
                    closing_rate = dot(los, target_vel - X(4:6)) / range_now;
                    if closing_rate >= 0
                        is_term = true; reason = 'PassedTarget'; return;
                    end
                end
            end
            if ~isempty(T.MaxDownrange) && (X(1) - target_pos(1)) > T.MaxDownrange
                is_term = true; reason = 'MaxDownrange'; return;
            end
        end
    end
end