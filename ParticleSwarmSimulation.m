%% File: ParticleSwarmSimulation.m
classdef ParticleSwarmSimulation < handle
    % Lockstep multi-body flight phase for a fragment/sub-munition swarm
    % released by Munition.spawnParticles at burst. Mirrors Simulation.run()'s
    % loop shape (advance target, log telemetry, check termination,
    % integrate) but per-particle: every still-alive particle is stepped
    % each dt, each checked against its own termination independently
    % ("meets the target" or "loses its chance", same criteria shape as
    % Simulation.Termination via the shared Simulation.evalTerminationCriteria),
    % and the whole phase stops once every particle is done or MaxTime hits.
    % Run lockstep (not one particle fully simulated after another) so a
    % live Visualizer actually shows the swarm dispersing together in time.
    properties
        Particles   cell         = {}
        Target                   = []
        Environment              = []
        Integrator               = RK4FixedStep()
        TimeSpan    (1,2) double = [0 15]
        Dt          (1,1) double = 0.001
        % PassedTarget is the primary "lost its chance" signal here (a
        % fragment stops the moment it's no longer closing on the target -
        % see Simulation.evalTerminationCriteria); MaxTime/MaxDownrange are
        % generous backstops in case a fragment somehow never trips it
        % (e.g. ends up on a near-parallel course), not the everyday
        % stopping condition.
        Termination struct = struct(...
            'MaxTime',        12.0, ...
            'ImpactDistance', 2.0, ...
            'MinAltitude',    0, ...
            'MaxAltitude',    [], ...
            'MaxDownrange',   500.0, ...
            'PassedTarget',   true)
        Visualizer  = []
        States      cell    = {}   % current state vector per particle, valid during/after run()
        Alive       logical = []   % true while a particle hasn't terminated yet
        Telemetry   cell    = {}   % per-particle telemetry struct (time/pos/vel/TerminationReason), after run()
    end

    methods
        function obj = ParticleSwarmSimulation(particles, target, env, timespan, dt)
            if nargin > 0, obj.Particles = particles; end
            if nargin > 1, obj.Target = target; end
            if nargin > 2, obj.Environment = env; end
            if nargin > 3, obj.TimeSpan = timespan; end
            if nargin > 4, obj.Dt = dt; end
        end

        function tel = run(obj)
            n  = numel(obj.Particles);
            dt = obj.Dt;
            time_array = obj.TimeSpan(1):dt:obj.TimeSpan(2);
            N_steps = numel(time_array);

            obj.States = cell(n, 1);
            for i = 1:n
                obj.States{i} = obj.Particles{i}.Dynamics.InitialState;
                obj.Particles{i}.Dynamics.CurrentState = obj.States{i};
            end
            obj.Alive = true(n, 1);

            tel = cell(n, 1);
            for i = 1:n
                tel{i}.time = zeros(N_steps, 1);
                tel{i}.pos  = zeros(N_steps, 3);
                tel{i}.vel  = zeros(N_steps, 3);
                tel{i}.steps = 0;
                tel{i}.TerminationReason = '';
            end

            tgt_bus = obj.resolveTargetBus();

            if ~isempty(obj.Visualizer)
                obj.Visualizer.init(obj.States, tgt_bus, obj.Alive);
            end

            for k = 1:N_steps
                t = time_array(k);

                if isobject(obj.Target) && ismethod(obj.Target, 'step')
                    obj.Target.step(t, dt, obj.meanAlivePosition());
                end
                tgt_bus = obj.resolveTargetBus();

                for i = 1:n
                    if ~obj.Alive(i), continue; end
                    X = obj.States{i};

                    s = tel{i}.steps + 1;
                    tel{i}.steps  = s;
                    tel{i}.time(s) = t;
                    tel{i}.pos(s, :) = X(1:3)';
                    tel{i}.vel(s, :) = X(4:6)';

                    [is_term, reason] = Simulation.evalTerminationCriteria(obj.Termination, t, X, tgt_bus.pos, tgt_bus.vel);
                    if is_term
                        obj.Alive(i) = false;
                        tel{i}.TerminationReason = reason;
                        continue;
                    end

                    obj.Particles{i}.Dynamics.CurrentState = X;
                    X_next = obj.Integrator.step(...
                        @(tt, XX) obj.Particles{i}.getDerivatives(tt, XX, obj.Environment), t, X, dt);
                    obj.States{i} = X_next;
                end

                if ~isempty(obj.Visualizer)
                    obj.Visualizer.step(t, obj.States, tgt_bus, obj.Alive);
                end

                if ~any(obj.Alive)
                    break;
                end
            end

            for i = 1:n
                s = tel{i}.steps;
                tel{i}.time = tel{i}.time(1:s);
                tel{i}.pos  = tel{i}.pos(1:s, :);
                tel{i}.vel  = tel{i}.vel(1:s, :);
            end
            obj.Telemetry = tel;
        end
    end

    methods (Access = private)
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

        function pos = meanAlivePosition(obj)
            alive_idx = find(obj.Alive);
            if isempty(alive_idx)
                pos = [0; 0; 0];
                return;
            end
            acc = [0; 0; 0];
            for i = alive_idx'
                acc = acc + obj.States{i}(1:3);
            end
            pos = acc / numel(alive_idx);
        end
    end
end
