%% Scenario: 35mm HE Fragmentation Burst vs. Anti-Drone Target
% Same engagement geometry as runner_35mm_antidrone.m, but the round is
% given a BurstTime + ParticleModel: shortly before the predicted intercept,
% the single-body flight stops and a swarm of fragments (see
% ParticleModelFactory.createForward35mm) splits off, each flying its own
% drag+gravity-only flight (see Particle/AeroParticle) until it meets the
% target or loses its chance. A second live, zoomed-in plot
% (ParticleSwarmPlotter) shows the swarm dispersing.
clear; clc; clf;

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

% 4. Fire Control: analytical direct-lead solve (same as runner_35mm_antidrone)
fcc = FireControlComputer(env.Gravity, 'Z-Up');
v0 = round.Charge.MuzzleVelocity;
launcher_pos = [0; 0; 0];

sol = fcc.solveFiringAngles(launcher_pos, v0, target.getTargetBus(), 'DirectLead');
if ~sol.IsValid
    error('No valid direct-lead solution (target out of envelope or geometry infeasible).');
end

fprintf('=== Direct-Lead Solution ===\n');
fprintf('Elevation                : %.1f mils\n', sol.Elevation);
fprintf('Azimuth                  : %.1f mils\n', sol.Azimuth);
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
round.Dynamics.InitialState = round.buildLaunchState(launcher_pos, v0, ...
    FireControlComputer.mil2rad(sol.Elevation), FireControlComputer.mil2rad(sol.Azimuth));

round.BurstTime = FireControlComputer.solveBurstTimeForStandoff(...
    @MunitionFactory.create35mmHE, launcher_pos, v0, sol.Elevation, sol.Azimuth, ...
    target, env, STANDOFF_RANGE, [0 5], 0.001, 3.0, 100.0);
fprintf('Burst standoff time      : %.3f s (target standoff = %.1f m)\n', round.BurstTime, STANDOFF_RANGE);

% 6. Assemble Simulation & Execute
sim = Simulation(round, target, env, [0 5], 0.001);
sim.Frame = 'FDR';
sim.Termination.ImpactDistance = 3.0;
sim.Termination.MaxDownrange   = 100.0;
sim.Visualizer = LiveEngagementPlotter(5);

% Post-burst swarm phase settings. ParticleTermination is left at
% ParticleSwarmSimulation's own default (PassedTarget=true is the real
% stopping condition - each fragment ends as soon as it's no longer
% closing on the target, rather than running out a fixed clock), just
% with a tighter ImpactDistance suited to these small fragments.
sim.ParticleVisualizer = ParticleSwarmPlotter(5);
sim.ParticleTermination = struct('MaxTime', 8.0, 'ImpactDistance', 0.05, ...
    'MinAltitude', 0, 'MaxAltitude', [], 'MaxDownrange', 20.0, 'PassedTarget', true);

% A 5 cm hit sphere is smaller than one fixed dt=0.001 step at fragment
% speed (0.001 s * a few hundred m/s = tens of cm) - a sample could easily
% straddle the target without ever landing inside it. Shrinks dt for the
% whole swarm once its closest fragment is within EngageRange, down to a
% floor fine enough that even a fast fragment samples well inside 5 cm.
adaptive_dt = RK4AdaptiveNearEncounter();
adaptive_dt.EngageRange = STANDOFF_RANGE + 5.0;
adaptive_dt.MinDt       = 2e-5; % ~2 cm spacing at 1000 m/s
sim.ParticleIntegrator = adaptive_dt;

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
