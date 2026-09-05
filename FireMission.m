classdef FireMission < handle
    % Fires NumRounds from a WeaponPlatform at a continuously-tracked
    % moving Target, with realistic inter-round timing: each round's lead
    % solve uses the target's actual position at THAT round's fire time
    % (Target.step is advanced by 60/RateOfFire between rounds), not a
    % single lead frozen at t=0. Simulation stays a single-trajectory
    % integrator - this class is a thin loop on top of it, building a
    % fresh Munition per round (via MunitionBuilder) so no per-round GNC
    % state can leak between rounds.
    properties
        Platform                       % WeaponPlatform
        MunitionBuilder                 % function handle, e.g. @() MunitionFactory.create35mmHE()
        Target                          % BaseTarget subclass, advanced across the whole mission
        Environment                    % Environment
        LauncherPos     (3,1) double = [0;0;0]
        FireMode        (1,:) char   = 'DirectLead' % passed to FireControlComputer.solveFiringAngles
        NumRounds       (1,1) double = 1
        Dt              (1,1) double = 0.005
        TimeOfFlightPad (1,1) double = 2.0   % Simulation TimeSpan = predicted TOF * this
        ImpactDistance  (1,1) double = 3.0   % m, per-round hit threshold
        MaxDownrange    (1,1) double = 100.0 % m, per-round flyby/overshoot threshold
    end

    methods
        function obj = FireMission(platform, munition_builder, target, env)
            if nargin > 0, obj.Platform = platform; end
            if nargin > 1, obj.MunitionBuilder = munition_builder; end
            if nargin > 2, obj.Target = target; end
            if nargin > 3, obj.Environment = env; end
        end

        function results = run(obj)
            fcc = FireControlComputer(obj.Environment.Gravity, 'Z-Up');
            fcc.ElevationMin = obj.Platform.ElevationMin;
            fcc.ElevationMax = obj.Platform.ElevationMax;
            fcc.AzimuthMin   = obj.Platform.AzimuthMin;
            fcc.AzimuthMax   = obj.Platform.AzimuthMax;

            dt_rounds = 60.0 / obj.Platform.RateOfFire; % seconds between successive fire times
            t_mission = 0;

            results = struct('RoundNum', {}, 'FireTime', {}, 'IsValid', {}, ...
                'Elevation', {}, 'Azimuth', {}, 'MuzzleVelocity', {}, ...
                'Telemetry', {}, 'MissDistance', {}, 'TerminationReason', {});

            for i = 1:obj.NumRounds
                if i > 1
                    % Advance the live target track through the inter-
                    % round gap (independent of any previous round's own
                    % flight time - RateOfFire governs time between
                    % trigger pulls, not between impacts).
                    obj.Target.step(t_mission, dt_rounds, []);
                    t_mission = t_mission + dt_rounds;
                end

                tgt_bus = obj.Target.getTargetBus();
                v0_round = obj.Platform.MuzzleVelocity + obj.Platform.MuzzleVelocityStdDev * randn();

                sol = fcc.solveFiringAngles(obj.LauncherPos, v0_round, tgt_bus, obj.FireMode);

                r = struct('RoundNum', i, 'FireTime', t_mission, 'IsValid', sol.IsValid, ...
                    'Elevation', NaN, 'Azimuth', NaN, 'MuzzleVelocity', v0_round, ...
                    'Telemetry', [], 'MissDistance', NaN, 'TerminationReason', '');

                if ~sol.IsValid
                    results(end+1) = r; %#ok<AGROW>
                    continue;
                end

                % Per-round aiming jitter, clamped to the platform's
                % mechanical stops (a real mount can't point past them).
                elev = sol.Elevation + obj.Platform.ElevationJitterStdDev * randn();
                azim = sol.Azimuth   + obj.Platform.AzimuthJitterStdDev * randn();
                elev = max(min(elev, obj.Platform.ElevationMax), obj.Platform.ElevationMin);
                azim = max(min(azim, obj.Platform.AzimuthMax), obj.Platform.AzimuthMin);

                munition = obj.MunitionBuilder();
                munition.Dynamics.InitialState = munition.Dynamics.buildLaunchState(...
                    obj.LauncherPos, v0_round, elev, azim, munition.MassProperties.Mass);

                sim = Simulation(munition, obj.Target, obj.Environment, ...
                    [0, sol.TimeOfFlight * obj.TimeOfFlightPad], obj.Dt);
                sim.Termination.ImpactDistance = obj.ImpactDistance;
                sim.Termination.MaxDownrange   = obj.MaxDownrange;

                telemetry = sim.run();

                final_pos = telemetry.pos(end, :)';
                final_tgt = telemetry.target_pos(end, :)';

                r.Elevation         = elev;
                r.Azimuth           = azim;
                r.Telemetry         = telemetry;
                r.MissDistance      = norm(final_pos - final_tgt);
                r.TerminationReason = sim.TerminationReason;

                results(end+1) = r; %#ok<AGROW>
            end
        end
    end
end
