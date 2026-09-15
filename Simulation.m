%% File: Simulation.m
classdef Simulation < handle
    properties
        Munition                  = []  % Munition container OR raw BaseDynamics object
        Target                    = []  % TargetBase subclass OR struct
        Environment               = []  % Environment object
        Frame     (1,:) char      = 'Z-Up'  % 'Z-Up' or 'NED' - see CoordinateFrame; propagated to Munition.Dynamics/Aero/Visualizer/the burst swarm at run()
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
        %
        % MinAltitude defaults to NaN, not a fixed 0: run() resolves NaN
        % to the launch altitude (CoordinateFrame.altitude of the initial
        % state) once, before the main loop - so a munition launched from
        % altitude 600 terminates on returning to 600, one launched from 0
        % terminates at 0, with no per-scenario setup needed either way.
        % Set MinAltitude explicitly (to any real number, or [] to disable
        % ground-impact termination entirely) to override this.
        Termination struct = struct(...
            'MaxTime',        [], ...   % s;  stop once t >= MaxTime
            'ImpactDistance', [], ...   % m;  stop once within this of target
            'MinAltitude',    NaN, ...  % m;  stop on descending through this - NaN = launch altitude, resolved in run()
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
        ParticleIntegrator     = []  % [] = ParticleSwarmSimulation's own default (RK4FixedStep)
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
            % Propagate the chosen Frame to every sub-object that needs to
            % agree on it, so a runner sets obj.Frame in exactly one place
            % rather than on each of Dynamics/Aero/Visualizer separately.
            if isprop(obj.Munition, 'Dynamics') && isprop(obj.Munition.Dynamics, 'Frame')
                obj.Munition.Dynamics.Frame = obj.Frame;
            end
            if isprop(obj.Munition, 'Aero') && isprop(obj.Munition.Aero, 'Frame')
                obj.Munition.Aero.Frame = obj.Frame;
            end
            if ~isempty(obj.Visualizer) && isprop(obj.Visualizer, 'Frame')
                obj.Visualizer.Frame = obj.Frame;
            end

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

            % Resolve MinAltitude's NaN default (see Termination's own
            % comment) to this run's actual launch altitude, once, before
            % any termination check can run.
            if isnan(obj.Termination.MinAltitude)
                obj.Termination.MinAltitude = CoordinateFrame.altitude(X, obj.Frame);
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
                alt_x    = CoordinateFrame.altitude(X, obj.Frame);
                alt_tgt  = CoordinateFrame.altitude(tgt_bus.pos, obj.Frame);
                bounds = [-100, max(X(1), tgt_bus.pos(1)) + 100, 0, max(alt_x, alt_tgt) + 300];
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
            tel.Frame = obj.Frame; % lets plotters read altitude back out correctly regardless of Frame
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
            % SelfDestructTime is a hard ceiling on when that burst
            % happens: it fires even if BurstTime was never programmed,
            % and preempts BurstTime if BurstTime would fire later -
            % the round can't fly on past it as an unexploded dud.
            if isa(obj.Munition, 'Munition')
                t_burst = obj.Munition.BurstTime;
                if ~isempty(obj.Munition.SelfDestructTime) && ...
                        (isempty(t_burst) || t_burst > obj.Munition.SelfDestructTime)
                    t_burst = obj.Munition.SelfDestructTime;
                end
                if ~isempty(t_burst) && t >= t_burst
                    is_term = true; reason = 'Burst'; return;
                end
            end
            [is_term, reason] = Simulation.evalTerminationCriteria(obj.Termination, t, X, target_pos, target_vel, obj.Frame);
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
            swarm.Frame = obj.Frame;
            if ~isempty(fieldnames(obj.ParticleTermination))
                swarm.Termination = obj.ParticleTermination;
            end
            swarm.Visualizer = obj.ParticleVisualizer;
            if ~isempty(obj.ParticleIntegrator)
                swarm.Integrator = obj.ParticleIntegrator;
            end
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
            % Target objects are always built/stepped in plain "Z-Up"
            % numbers (see TargetBase/TargetStatic/TargetConstantVelocity -
            % none of them know about Frame), so the bus handed to the
            % munition's physics - which DOES run in obj.Frame - is
            % rotated here, at the single point of consumption, rather
            % than requiring every runner to pre-rotate a target's
            % construction numbers by hand.
            if isobject(obj.Target) && ismethod(obj.Target, 'getTargetBus')
                tgt_bus = obj.Target.getTargetBus();
            elseif isstruct(obj.Target)
                tgt_bus = obj.Target;
            else
                tgt_bus = struct('pos', [0;0;0], 'vel', [0;0;0], 'accel', [0;0;0], ...
                                 'LaserReflectivity', 0.3, 'ThermalSignature', 300, 'RCS', 1.0);
            end
            tgt_bus.pos = CoordinateFrame.rotateVector(tgt_bus.pos, obj.Frame);
            tgt_bus.vel = CoordinateFrame.rotateVector(tgt_bus.vel, obj.Frame);
            if isfield(tgt_bus, 'accel')
                tgt_bus.accel = CoordinateFrame.rotateVector(tgt_bus.accel, obj.Frame);
            end
        end
    end

    methods (Static)
        function [is_term, reason] = evalTerminationCriteria(T, t, X, target_pos, target_vel, frame)
            % Shared by Simulation.checkTermination and
            % ParticleSwarmSimulation (each fragment is checked against the
            % same criteria shape - ImpactDistance = "meets the target",
            % MinAltitude/MaxDownrange/PassedTarget = "loses its chance").
            % Evaluates each enabled criterion in T (any field left []/false
            % is skipped) and stops on the first one that trips.
            % ImpactDistance/PassedTarget/MaxDownrange are norms/dot
            % products/the downrange component - unaffected by frame as
            % long as target_pos/target_vel are already expressed in the
            % same frame as X (see Simulation.resolveTargetBus).
            % MinAltitude/MaxAltitude go through CoordinateFrame.altitude
            % since "altitude" isn't simply X(3) under every frame.
            % T.MinAltitude is assumed already resolved to a real number
            % (or []) by this point - Simulation.run()/ParticleSwarmSimulation.
            % run() each do that once, before their loop starts, from
            % their own default NaN ("use launch altitude") - a NaN
            % reaching the comparison below would silently never trigger
            % (NaN comparisons are always false), so don't call this with
            % an unresolved Termination struct.
            if nargin < 6, frame = 'Z-Up'; end
            is_term = false;
            reason  = '';

            if ~isempty(T.MaxTime) && t >= T.MaxTime
                is_term = true; reason = 'MaxTime'; return;
            end
            if ~isempty(T.ImpactDistance) && norm(target_pos - X(1:3)) < T.ImpactDistance
                is_term = true; reason = 'ImpactDistance'; return;
            end
            altitude = CoordinateFrame.altitude(X, frame);
            if ~isempty(T.MinAltitude) && altitude < T.MinAltitude && norm(X(4:6)) > 0
                is_term = true; reason = 'MinAltitude'; return;
            end
            if ~isempty(T.MaxAltitude) && altitude > T.MaxAltitude
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