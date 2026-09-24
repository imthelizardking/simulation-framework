%% Scenario: 25mm Natural-Fragmentation Burst vs. Anti-Drone Target
% Same shape as runner_35mm_fragmentation.m, scaled down to the 25mm round:
% M793 is historically a TP-T (target-practice tracer) round with no HE
% filler, so real-world it wouldn't fragment at all - it's treated here as
% if HE-filled purely to exercise the fragmentation model, same
% illustrative-only caveat MunitionFactory already carries for its other
% presets. Unlike the 35mm HE-T case (still just casing break-up, not
% pre-formed sub-munitions), this one is explicitly "natural fragmentation"
% - the shell body itself splitting into small fragments at burst, not a
% deliberately engineered payload - so it reuses the same basic
% ForwardExplosionParticleModel/ParticleModelFactory.createForward25mm
% (rings of small cylindrical casing fragments), just at 25mm scale and a
% tighter burst standoff.
clear; clc; clf;

% 1. Setup Environment (standard atmosphere, no wind)
env = Environment();

% 2. Setup Target: small drone at a shorter engagement range than the 35mm
%    scenario, in keeping with 25mm's smaller HE-equivalent fragmentation
%    effect
% forward=700, down=-100 (100m above launch), right=-150 (150m left);
% vel forward=-10 (closing), down=0 (level), right=-6. Same physical
% scenario as before (forward=700,crossrange=150,altitude=100), just
% re-authored directly in FDR (see TargetBase's constructor comment).
target = TargetConstantVelocity([700; -100; -150], [-10; 0; -6], 'FDR');

% 3. Instantiate Munition via MunitionFactory, attach the fragmentation model
round = MunitionFactory.create25mmM793();
round.ParticleModel = ParticleModelFactory.createForward25mm();

% 4. Fire Control: analytical direct-lead solve (same approach as
%    runner_35mm_fragmentation.m)
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
%    STANDOFF_RANGE meters from the target at burst - found the same way
%    as runner_35mm_fragmentation.m, off a "recon" flight of the same
%    unburst round/target rather than a guessed time offset (solveDirectLead
%    ignores drag, so its predicted TimeOfFlight doesn't land on a fixed
%    standoff distance reliably).
STANDOFF_RANGE = 10.0; % m; burst this far short of the target
round.Dynamics.InitialState = round.buildLaunchState(launcher_pos, v0, ...
    FireControlComputer.mil2rad(sol.Elevation), FireControlComputer.mil2rad(sol.Azimuth));

round.BurstTime = FireControlComputer.solveBurstTimeForStandoff(...
    @MunitionFactory.create25mmM793, launcher_pos, v0, sol.Elevation, sol.Azimuth, ...
    target, env, STANDOFF_RANGE, [0 5], 0.001, 2.0, 100.0);
fprintf('Burst standoff time      : %.3f s (target standoff = %.1f m)\n', round.BurstTime, STANDOFF_RANGE);

% 6. Assemble Simulation & Execute
sim = Simulation(round, target, env, [0 5], 0.001);
sim.Frame = 'FDR';
sim.Termination.ImpactDistance = 2.0;
sim.Termination.MaxDownrange   = 100.0;
sim.Visualizer = LiveEngagementPlotter(5);

% Post-burst swarm phase settings. Left at ParticleSwarmSimulation's own
% default (PassedTarget=true is the real stopping condition), just with a
% tighter ImpactDistance suited to these small fragments.
sim.ParticleVisualizer = ParticleSwarmPlotter(5);
sim.ParticleTermination = struct('MaxTime', 8.0, 'ImpactDistance', 0.05, ...
    'MinAltitude', 0, 'MaxAltitude', [], 'MaxDownrange', 20.0, 'PassedTarget', false);

% See runner_35mm_fragmentation.m's comment on this - a 5 cm hit sphere is
% smaller than a fixed dt=0.001 step at fragment speed, so dt must shrink
% near the target for the check to ever have a chance of landing inside it.
adaptive_dt = RK4AdaptiveNearEncounter();
adaptive_dt.EngageRange = STANDOFF_RANGE + 5.0;
adaptive_dt.MinDt       = 2e-5; % ~2 cm spacing at 1000 m/s
sim.ParticleIntegrator = adaptive_dt;

fprintf('Firing 25mm round (burst at t=%.3f s)...\n', round.BurstTime);
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
