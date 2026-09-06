%% Scenario: 35mm HE Fragmentation Burst vs. Anti-Drone Target
% Same engagement geometry as runner_35mm_antidrone.m, but the round is
% given a BurstTime + ParticleModel: shortly before the predicted intercept,
% the single-body flight stops and a swarm of fragments (see
% ParticleModelFactory.createForward35mm) splits off, each flying its own
% drag+gravity-only flight (see Particle/AeroParticle) until it meets the
% target or loses its chance. A second live, zoomed-in plot
% (ParticleSwarmPlotter) shows the swarm dispersing.
clear; clc; close all;

% 1. Setup Environment (standard atmosphere, no wind)
env = Environment(1.225, 9.81, [0; 0; 0]);

% 2. Setup Target: same DJI-class drone as runner_35mm_antidrone.m
target = TargetConstantVelocity([1000; 200; 120], [-12; 8; 0]);

% 3. Instantiate Munition via MunitionFactory, attach the fragmentation model
round = MunitionFactory.create35mmHE();
round.ParticleModel = ParticleModelFactory.createForward35mm();

% 4. Fire Control: analytical direct-lead solve (same as runner_35mm_antidrone)
fcc = FireControlComputer(env.Gravity, 'Z-Up');
v0 = round.Charge.MuzzleVelocity;
launcher_pos = [0; 0; 0];

sol = fcc.solveFiringAngles(launcher_pos, v0, target.getTargetBus(), 'DirectLead');
if ~sol.IsValid
    error('No valid direct-lead solution (target out of envelope or geometry infeasible).');
end

fprintf('=== Direct-Lead Solution ===\n');
fprintf('Elevation                : %.2f deg\n', rad2deg(sol.Elevation));
fprintf('Azimuth                  : %.2f deg\n', rad2deg(sol.Azimuth));
fprintf('Predicted Intercept Time : %.3f s\n', sol.TimeOfFlight);

% 5. Apply Solved Initial State. BurstTime is set so the round is exactly
%    STANDOFF_RANGE meters from the target at burst, not a guessed time
%    offset: solveDirectLead's quadratic ignores drag (see
%    runner_35mm_antidrone.m's own comment on this), so the round's real,
%    drag-decelerated arrival is later than sol.TimeOfFlight predicts and a
%    fixed time offset wouldn't reliably land on a fixed standoff distance.
%    Instead, a "recon" flight of the same unburst round/target is run
%    first purely to find the real time-vs-range curve, and BurstTime is
%    picked off of that.
STANDOFF_RANGE = 25.0; % m; burst this far short of the target
round.Dynamics.InitialState = round.buildLaunchState(launcher_pos, v0, sol.Elevation, sol.Azimuth);

recon_round  = MunitionFactory.create35mmHE();
recon_round.Dynamics.InitialState = recon_round.buildLaunchState(launcher_pos, v0, sol.Elevation, sol.Azimuth);
recon_target = TargetConstantVelocity(target.Position, target.Velocity);

recon_sim = Simulation(recon_round, recon_target, env, [0 5], 0.001);
recon_sim.Termination.ImpactDistance = 3.0;
recon_sim.Termination.MaxDownrange   = 100.0;
recon_tel = recon_sim.run();

round.BurstTime = findRangeCrossingTime(recon_tel, STANDOFF_RANGE);
fprintf('Burst standoff time      : %.3f s (range = %.1f m at that instant)\n', ...
    round.BurstTime, interp1(recon_tel.time, recon_tel.range, round.BurstTime));

% 6. Assemble Simulation & Execute
sim = Simulation(round, target, env, [0 5], 0.001);
sim.Termination.ImpactDistance = 3.0;
sim.Termination.MaxDownrange   = 100.0;
sim.Visualizer = LiveEngagementPlotter(5);

% Post-burst swarm phase settings. ParticleTermination is left at
% ParticleSwarmSimulation's own default (PassedTarget=true is the real
% stopping condition - each fragment ends as soon as it's no longer
% closing on the target, rather than running out a fixed clock), just
% with a tighter ImpactDistance suited to these small fragments.
sim.ParticleVisualizer = ParticleSwarmPlotter(5);
sim.ParticleTermination = struct('MaxTime', 12.0, 'ImpactDistance', 2.5, ...
    'MinAltitude', 0, 'MaxAltitude', [], 'MaxDownrange', 500.0, 'PassedTarget', true);

fprintf('Firing 35mm round (burst at t=%.3f s)...\n', round.BurstTime);
tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.3f s simulated).\n', toc, telemetry.time(end));

fprintf('Parent Termination Reason : %s\n', sim.TerminationReason);

% 7. Post-Burst Fragment Report
if ~isempty(sim.ParticleSwarm)
    swarm = sim.ParticleSwarm;
    n = numel(swarm.Telemetry);
    hits = 0;
    fprintf('\n%6s %20s %10s\n', 'Frag#', 'Reason', 'FinalX/Z');
    for i = 1:n
        frag_tel = swarm.Telemetry{i};
        if strcmp(frag_tel.TerminationReason, 'ImpactDistance'), hits = hits + 1; end
        if ~isempty(frag_tel.pos)
            fprintf('%6d %20s %6.1f/%.1f\n', i, frag_tel.TerminationReason, frag_tel.pos(end,1), frag_tel.pos(end,3));
        end
    end
    fprintf('\nFragments that reached the target: %d / %d\n', hits, n);
else
    fprintf('Munition never reached BurstTime before another termination criterion tripped.\n');
end

function t_burst = findRangeCrossingTime(telemetry, standoff)
    % Finds the (interpolated) time at which telemetry.range last crosses
    % down through `standoff` meters - i.e. the instant the round is
    % `standoff` meters from the target, still closing toward intercept.
    range_ = telemetry.range;
    time_  = telemetry.time;
    idx = find(range_(1:end-1) >= standoff & range_(2:end) < standoff, 1, 'last');
    if isempty(idx)
        error('findRangeCrossingTime:NotFound', ...
            'Range never crossed %.1f m during the recon flight (min range reached = %.1f m).', ...
            standoff, min(range_));
    end
    r0 = range_(idx);   r1 = range_(idx+1);
    t0 = time_(idx);    t1 = time_(idx+1);
    frac = (r0 - standoff) / (r0 - r1);
    t_burst = t0 + frac * (t1 - t0);
end
