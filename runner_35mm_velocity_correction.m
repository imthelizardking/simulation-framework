%% Scenario: 35mm Smart-Fuze Burst-Time Correction from Measured Muzzle Velocity
% 35mm counterpart to runner_25mm_velocity_correction.m - see that file
% for the full explanation of the algorithm. Same engagement geometry as
% runner_35mm_fragmentation.m/runner_35mm_antidrone.m, but the fuze's
% burst time is first programmed from the charge's NOMINAL muzzle
% velocity (before this round has actually been fired), then corrected
% once the round's own measured muzzle velocity is known.
clear; clc; close all;

% 1. Setup Environment (standard atmosphere, no wind)
env = Environment();

% 2. Setup Target: same DJI-class drone as runner_35mm_antidrone.m
% forward=1000, down=-120 (120m above launch), right=-200 (200m left);
% vel forward=-12 (closing), down=0 (level), right=-8. Same physical
% scenario as before (forward=1000,crossrange=200,altitude=120), just
% re-authored directly in FDR (see TargetBase's constructor comment).
target = TargetConstantVelocity([1000; -120; -200], [-12; 0; -8], 'FDR');

% 3. Instantiate Munition via MunitionFactory, attach the fragmentation model
round = MunitionFactory.create35mmHE();
round.ParticleModel = ParticleModelFactory.createForward35mm();
builder = @MunitionFactory.create35mmHE;

% 4. Fire Control: analytical direct-lead solve using the charge's
%    NOMINAL (rated) muzzle velocity - the best estimate available when
%    aiming, before this specific round has actually been fired.
fcc = FireControlComputer(env.Gravity, 'Z-Up');
v0_nominal = round.Charge.MuzzleVelocity;
launcher_pos = [0; 0; 0];

sol = fcc.solveFiringAngles(launcher_pos, v0_nominal, target.getTargetBus(), 'DirectLead');
if ~sol.IsValid
    error('No valid direct-lead solution (target out of envelope or geometry infeasible).');
end

fprintf('=== Direct-Lead Solution (nominal v0 = %.1f m/s) ===\n', v0_nominal);
fprintf('Elevation                : %.1f mils\n', sol.Elevation);
fprintf('Azimuth                  : %.1f mils\n', sol.Azimuth);

% 5. Pre-fire (programmed) burst time: the fuze setting the fire control
%    computer transmits BEFORE knowing this round's true muzzle
%    velocity, aimed at bursting STANDOFF_RANGE meters short of the
%    target.
STANDOFF_RANGE = 25.0; % m; burst this far short of the target
timespan = [0 5]; dt = 0.001; impact_dist = 3.0; max_downrange = 100.0;

nominal_burst_time = FireControlComputer.solveBurstTimeForStandoff(...
    builder, launcher_pos, v0_nominal, sol.Elevation, sol.Azimuth, ...
    target, env, STANDOFF_RANGE, timespan, dt, impact_dist, max_downrange);
fprintf('Programmed burst time (pre-fire, nominal v0) : %.3f s\n', nominal_burst_time);

% 6. Fire the round: its true muzzle velocity is drawn from the charge's
%    own round-to-round dispersion (same draw FireMission.run() uses) -
%    the elevation/azimuth solved above are already locked in by the
%    time this is known.
actual_v0 = round.Charge.MuzzleVelocity + round.Charge.MuzzleVelocityStdDev * randn();
fprintf('\nRound fired. Muzzle velocity sensor measures v0 = %.2f m/s (nominal was %.1f, delta %+.2f m/s)\n', ...
    actual_v0, v0_nominal, actual_v0 - v0_nominal);

% 7. Burst-time correction: same standoff solve, re-run with the now-
%    known actual velocity instead of the nominal one (same fixed
%    elevation/azimuth).
[corrected_burst_time, actual_tel] = FireControlComputer.solveBurstTimeForStandoff(...
    builder, launcher_pos, actual_v0, sol.Elevation, sol.Azimuth, ...
    target, env, STANDOFF_RANGE, timespan, dt, impact_dist, max_downrange);
fprintf('Corrected burst time (post-fire, measured v0): %.3f s (correction: %+.3f s)\n', ...
    corrected_burst_time, corrected_burst_time - nominal_burst_time);

% 8. What if the fuze DIDN'T get corrected? actual_tel above is the real
%    round's own trajectory (same actual_v0/elevation/azimuth as the
%    live shot below), so the standoff the uncorrected pre-fire burst
%    time would have actually produced is just a lookup on it - no need
%    for a second simulation.
uncorrected_standoff = interp1(actual_tel.time, actual_tel.range, nominal_burst_time);
fprintf('Standoff range WITHOUT correction : %.2f m (intended %.1f m)\n', uncorrected_standoff, STANDOFF_RANGE);
fprintf('Standoff range WITH correction    : %.2f m (by construction)\n', STANDOFF_RANGE);

% 9. Fire the real round with the corrected burst time and run the full
%    engagement, including the fragmentation swarm.
round.Dynamics.InitialState = round.buildLaunchState(launcher_pos, actual_v0, ...
    FireControlComputer.mil2rad(sol.Elevation), FireControlComputer.mil2rad(sol.Azimuth));
round.BurstTime = corrected_burst_time;

sim = Simulation(round, target, env, timespan, dt);
sim.Frame = 'FDR';
sim.Termination.ImpactDistance = impact_dist;
sim.Termination.MaxDownrange   = max_downrange;
sim.Visualizer = LiveEngagementPlotter(5);

% Post-burst swarm phase settings - see runner_35mm_fragmentation.m.
sim.ParticleVisualizer = ParticleSwarmPlotter(5);
sim.ParticleTermination = struct('MaxTime', 8.0, 'ImpactDistance', 0.05, ...
    'MinAltitude', 0, 'MaxAltitude', [], 'MaxDownrange', 20.0, 'PassedTarget', true);

adaptive_dt = RK4AdaptiveNearEncounter();
adaptive_dt.EngageRange = STANDOFF_RANGE + 5.0;
adaptive_dt.MinDt       = 2e-5; % ~2 cm spacing at 1000 m/s
sim.ParticleIntegrator = adaptive_dt;

fprintf('\nFiring 35mm round with corrected fuze setting (burst at t=%.3f s)...\n', round.BurstTime);
tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.3f s simulated).\n', toc, telemetry.time(end));

fprintf('Parent Termination Reason : %s\n', sim.TerminationReason);

% 10. Post-Burst Fragment Report
if ~isempty(sim.ParticleSwarm)
    swarm = sim.ParticleSwarm;
    n = numel(swarm.Telemetry);
    hits = 0;
    fprintf('\n%6s %20s %10s\n', 'Frag#', 'Reason', 'FinalX/Y');
    for i = 1:n
        frag_tel = swarm.Telemetry{i};
        if strcmp(frag_tel.TerminationReason, 'ImpactDistance'), hits = hits + 1; end
        if ~isempty(frag_tel.pos)
            fprintf('%6d %20s %6.1f/%.1f\n', i, frag_tel.TerminationReason, frag_tel.pos(end,1), frag_tel.pos(end,2));
        end
    end
    fprintf('\nFragments that reached the target: %d / %d\n', hits, n);
else
    fprintf('Munition never reached BurstTime before another termination criterion tripped.\n');
end
