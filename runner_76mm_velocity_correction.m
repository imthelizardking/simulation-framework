%% Scenario: 76mm Smart-Fuze Burst-Time Correction from Measured Muzzle Velocity
% 76mm counterpart to runner_25mm_velocity_correction.m - see that file
% for the full explanation of the algorithm. Same long-range engagement
% geometry as runner_76mm.m (its detailed Mach-dependent aero table,
% higher drag over a longer flight than the 25/35mm cases), but the
% fuze's burst time is first programmed from the charge's NOMINAL muzzle
% velocity, then corrected once the round's own measured muzzle velocity
% is known. Standoff/timespan are scaled up from the 25/35mm examples
% for 76mm HE's larger lethal radius and this scenario's much longer
% range and flight time.
clear; clc; clf;

% 1. Setup Environment (standard atmosphere, no wind)
env = Environment();

% 2. Setup Target: same inbound target as runner_76mm.m/runner_sam_engagement.m
% forward=8000, down=-3000 (3000m above launch), right=0; vel forward=-250
% (closing), down=0, right=0. Same physical scenario as before
% (forward=8000,crossrange=0,altitude=3000), just re-authored directly
% in FDR (see TargetBase's constructor comment).
target = TargetConstantVelocity([8000; -3000; 0], [-250; 0; 0], 'FDR');

% 3. Instantiate Munition via MunitionFactory, attach the fragmentation model
round = MunitionFactory.create76mmHE();
round.ParticleModel = ParticleModelFactory.createForward76mm();
builder = @MunitionFactory.create76mmHE;

% 4. Fire Control: trajectory shooting (real Simulation flights of this
%    round against the moving target, same dt/Frame as the live shot)
%    using the charge's NOMINAL (rated) muzzle velocity - the best
%    estimate available when aiming, before this specific round has
%    actually been fired.
fcc = FireControlComputer(env.Gravity, 'Z-Up');
v0_nominal = round.Charge.MuzzleVelocity;
launcher_pos = [0; 0; 0];
timespan = [0 30]; dt = 0.005; impact_dist = 5.0; max_downrange = 300.0;

sol = fcc.solveTrajectoryShooting(builder, launcher_pos, v0_nominal, target, env, timespan, dt, 'FDR');
if ~sol.IsValid
    error('No valid trajectory solution (target out of envelope or geometry infeasible).');
end

fprintf('=== Trajectory Shooting Solution (nominal v0 = %.1f m/s) ===\n', v0_nominal);
fprintf('Elevation                : %.1f mils\n', sol.Elevation);
fprintf('Azimuth                  : %.1f mils\n', sol.Azimuth);
fprintf('Predicted Miss Distance  : %.3f m (%d iterations)\n', sol.MissDistance, sol.Iterations);

% 5. Pre-fire (programmed) burst time: the fuze setting the fire control
%    computer transmits BEFORE knowing this round's true muzzle
%    velocity, aimed at bursting STANDOFF_RANGE meters short of the
%    target. Longer range/flight time and a larger HE lethal radius than
%    the 25/35mm cases, so timespan/standoff/tolerances are scaled up to
%    match (see runner_76mm.m's own ImpactDistance=5.0/MaxDownrange=200
%    for the same reasoning).
STANDOFF_RANGE = 40.0; % m; burst this far short of the target

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
sim.Visualizer = LiveEngagementPlotter(20);

% Post-burst swarm phase settings - same shape as
% runner_35mm_fragmentation.m, just a wider EngageRange/backstop for the
% bigger standoff and higher fragment velocities at 76mm scale.
sim.ParticleVisualizer = ParticleSwarmPlotter(20);
sim.ParticleTermination = struct('MaxTime', 8.0, 'ImpactDistance', 0.1, ...
    'MinAltitude', 0, 'MaxAltitude', [], 'MaxDownrange', 40.0, 'PassedTarget', true);

adaptive_dt = RK4AdaptiveNearEncounter();
adaptive_dt.EngageRange = STANDOFF_RANGE + 10.0;
adaptive_dt.MinDt       = 2e-5; % ~2 cm spacing at 1000 m/s
sim.ParticleIntegrator = adaptive_dt;

fprintf('\nFiring 76mm round with corrected fuze setting (burst at t=%.3f s)...\n', round.BurstTime);
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
