%% Scenario: 25mm Smart-Fuze Burst-Time Correction from Measured Muzzle Velocity
% Demonstrates the basic feedback loop real programmable-fuze ammunition
% uses: FireControlComputer solves the firing angles and a NOMINAL burst
% time using the charge's rated (average) muzzle velocity, since this
% particular round's true muzzle velocity isn't known until it's actually
% fired. Once fired, a muzzle-velocity sensor (a coil/radar at the
% muzzle, standard on modern airburst systems) measures the round's
% ACTUAL velocity, which differs from nominal by the charge's normal
% round-to-round dispersion (Charge.MuzzleVelocityStdDev, same dispersion
% FireMission.run() already draws from). Elevation/azimuth are already
% fixed by then, but the fuze's burst timer can still be updated, so the
% burst time is recalculated - the same FireControlComputer.
% solveBurstTimeForStandoff helper used for the nominal estimate, just
% re-run with the measured velocity in place of the nominal one. That
% re-run is the whole "iterative" part of this very basic algorithm:
% redo the same calculation once better information is available, not a
% convergence loop.
clear; clc; clf;

% 1. Setup Environment (standard atmosphere, no wind)
env = Environment();

% 2. Setup Target: same DJI-class drone as runner_25mm_fragmentation.m
% forward=700, down=-100 (100m above launch), right=-150 (150m left);
% vel forward=-10 (closing), down=0 (level), right=-6. Same physical
% scenario as before (forward=700,crossrange=150,altitude=100), just
% re-authored directly in FDR (see TargetBase's constructor comment).
target = TargetConstantVelocity([700; -100; -150], [-10; 0; -6], 'FDR');

% 3. Instantiate Munition via MunitionFactory, attach the fragmentation model
round = MunitionFactory.create25mmM793();
round.ParticleModel = ParticleModelFactory.createForward25mm();
builder = @MunitionFactory.create25mmM793;

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
STANDOFF_RANGE = 10.0; % m; burst this far short of the target
timespan = [0 5]; dt = 0.001; impact_dist = 2.0; max_downrange = 100.0;

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

% Post-burst swarm phase settings - see runner_25mm_fragmentation.m.
sim.ParticleVisualizer = ParticleSwarmPlotter(5);
sim.ParticleTermination = struct('MaxTime', 8.0, 'ImpactDistance', 0.05, ...
    'MinAltitude', 0, 'MaxAltitude', [], 'MaxDownrange', 20.0, 'PassedTarget', false);

adaptive_dt = RK4AdaptiveNearEncounter();
adaptive_dt.EngageRange = STANDOFF_RANGE + 5.0;
adaptive_dt.MinDt       = 2e-5; % ~2 cm spacing at 1000 m/s
sim.ParticleIntegrator = adaptive_dt;

fprintf('\nFiring 25mm round with corrected fuze setting (burst at t=%.3f s)...\n', round.BurstTime);
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
