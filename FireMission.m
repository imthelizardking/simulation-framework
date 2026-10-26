classdef FireMission < handle
    % Fires NumRounds from a WeaponPlatform at a continuously-tracked
    % moving Target, with realistic inter-round timing: each round's lead
    % solve uses the target's actual position at THAT round's fire time
    % (Target.step is advanced by 60/RateOfFire between rounds), not a
    % single lead frozen at t=0. Simulation stays a single-trajectory
    % integrator - this class is a thin loop on top of it, building a
    % fresh Munition per round (via MunitionBuilder) so no per-round GNC
    % state can leak between rounds.
    %
    % Muzzle velocity is a property of the (munition, charge) pair, not
    % the weapon - MunitionBuilder must return a Munition with its Charge
    % set (see MunitionFactory/ChargeFactory). Platform.MaxMuzzleVelocity
    % is only the weapon's own rated ceiling; run() checks the charge
    % against it once, up front.
    properties
        Platform                       % WeaponPlatform
        MunitionBuilder                 % function handle, e.g. @() MunitionFactory.create35mmHE()
        Target                          % BaseTarget subclass, advanced across the whole mission
        Atmosphere                    % Atmosphere
        Frame           (1,:) char   = 'Z-Up' % 'Z-Up', 'NED', or 'FDR' - see CoordinateFrame; passed to each round's Simulation
        LauncherPos     (3,1) double = [0;0;0]
        % 'Trajectory' = FireControlComputer.solveTrajectoryShooting (real
        % Simulation flights of the round, same Frame/Dt as the shot);
        % anything else ('DirectLead', 'Indirect') is passed to
        % FireControlComputer.solveFiringAngles' analytic solvers.
        FireMode        (1,:) char   = 'Trajectory'
        MaxFlightTime   (1,1) double = 30.0  % s, bounds each solveTrajectoryShooting flight
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
            if nargin > 3, obj.Atmosphere = env; end
        end

        function results = run(obj)
            fcc = FireControlComputer(obj.Atmosphere.Gravity, 'Z-Up');
            fcc.ElevationMin = obj.Platform.ElevationMin;
            fcc.ElevationMax = obj.Platform.ElevationMax;
            fcc.AzimuthMin   = obj.Platform.AzimuthMin;
            fcc.AzimuthMax   = obj.Platform.AzimuthMax;

            % Every round in a mission uses the same munition+charge, so a
            % weapon/charge mismatch would fail identically every round -
            % check it once, up front, with a clear error instead of
            % repeating the same failure NumRounds times.
            probe = obj.MunitionBuilder();
            if isempty(probe.Charge)
                error('FireMission:NoCharge', 'Munition has no Charge assigned - MunitionBuilder must set one.');
            end
            if probe.Charge.MuzzleVelocity > obj.Platform.MaxMuzzleVelocity
                error('FireMission:ChargeIncompatible', ...
                    'Charge %s (%.1f m/s) exceeds %s''s rated max muzzle velocity (%.1f m/s).', ...
                    probe.Charge.Name, probe.Charge.MuzzleVelocity, obj.Platform.Name, obj.Platform.MaxMuzzleVelocity);
            end

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

                munition = obj.MunitionBuilder();
                % Must be set before buildLaunchState below - it reads
                % Dynamics.Frame at build time, before sim.Frame (set
                % further down) would otherwise propagate it.
                if isprop(munition.Dynamics, 'Frame'), munition.Dynamics.Frame = obj.Frame; end
                if isprop(munition.Aero, 'Frame'), munition.Aero.Frame = obj.Frame; end
                charge = munition.Charge;
                v0_round = charge.MuzzleVelocity + charge.MuzzleVelocityStdDev * randn();

                if strcmpi(obj.FireMode, 'Trajectory')
                    sol = fcc.solveTrajectoryShooting(obj.MunitionBuilder, obj.LauncherPos, v0_round, ...
                        tgt_bus, obj.Atmosphere, [0 obj.MaxFlightTime], obj.Dt, obj.Frame);
                else
                    sol = fcc.solveFiringAngles(obj.LauncherPos, v0_round, tgt_bus, obj.FireMode);
                end

                r = struct('RoundNum', i, 'FireTime', t_mission, 'IsValid', sol.IsValid, ...
                    'Elevation', NaN, 'Azimuth', NaN, 'MuzzleVelocity', v0_round, ...
                    'Telemetry', [], 'MissDistance', NaN, 'TerminationReason', '');

                if ~sol.IsValid
                    results(end+1) = r; %#ok<AGROW>
                    continue;
                end

                % Per-round aiming jitter, clamped to the platform's
                % mechanical stops (a real mount can't point past them).
                % elev/azim, like sol.Elevation/sol.Azimuth and the
                % Platform limits/jitter stddevs, are mils throughout -
                % buildLaunchState is the one place that still wants
                % radians, so convert only at that call.
                elev = sol.Elevation + obj.Platform.ElevationJitterStdDev * randn();
                azim = sol.Azimuth   + obj.Platform.AzimuthJitterStdDev * randn();
                elev = max(min(elev, obj.Platform.ElevationMax), obj.Platform.ElevationMin);
                azim = max(min(azim, obj.Platform.AzimuthMax), obj.Platform.AzimuthMin);

                munition.Dynamics.InitialState = munition.buildLaunchState(...
                    obj.LauncherPos, v0_round, ...
                    FireControlComputer.mil2rad(elev), FireControlComputer.mil2rad(azim));

                sim = Simulation(munition, obj.Target, obj.Atmosphere, ...
                    [0, sol.TimeOfFlight * obj.TimeOfFlightPad], obj.Dt);
                sim.Frame = obj.Frame;
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
