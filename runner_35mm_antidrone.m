%% Scenario: 35mm Anti-Drone Direct-Fire Engagement (Land-to-Air)
clear; clc; clf;

% 1. Setup Environment (standard atmosphere, no wind)
env = Environment();

% 2. Setup Target: small commercial quadcopter (DJI-class), level flight
%    at a typical cruise speed (~14 m/s here), crossing/closing on the gun
%    position at a modest patrol altitude.
% forward=1000, down=-120 (120m above launch), right=-200 (200m left);
% vel forward=-12 (closing), down=0 (level), right=-8. Same physical
% scenario as before (forward=1000,crossrange=200,altitude=120), just
% re-authored directly in FDR (see BaseTarget's constructor comment).
target = TargetConstantVelocity([1000; -120; -200], [-12; 0; -8], 'FDR');

% 3. Instantiate Munition via MunitionFactory
round = MunitionFactory.create35mmHE();

% 4. Fire Control: trajectory shooting. Unlike solveNumericalShooting
%    (built for a static target) it leads the moving target, and unlike
%    the analytic direct-lead solve (vacuum, constant bullet speed) every
%    candidate is a real Simulation flight of this round - same dt/Frame
%    as the live shot below - so the solved trajectory is the one flown.
fcc = FireControlComputer(env.Gravity, 'Z-Up');
v0 = round.Charge.MuzzleVelocity; % see MunitionFactory.create35mmHE
launcher_pos = [0; 0; 0];

sol = fcc.solveTrajectoryShooting(@MunitionFactory.create35mmHE, launcher_pos, v0, ...
    target, env, [0 5], 0.001, 'FDR');

if ~sol.IsValid
    error('No valid trajectory solution (target out of envelope or geometry infeasible).');
end

fprintf('=== Trajectory Shooting Solution ===\n');
fprintf('Elevation                : %.1f mils\n', sol.Elevation);
fprintf('Azimuth                  : %.1f mils\n', sol.Azimuth);
fprintf('Predicted Intercept Time : %.3f s\n', sol.TimeOfFlight);
fprintf('Predicted Miss Distance  : %.3f m (%d iterations)\n', sol.MissDistance, sol.Iterations);

% 5. Apply Solved Initial State
round.Dynamics.InitialState = round.buildLaunchState(launcher_pos, v0, ...
    FireControlComputer.mil2rad(sol.Elevation), FireControlComputer.mil2rad(sol.Azimuth));

% 6. Assemble Simulation & Execute. Unguided direct-fire round with no
%    terminal correction - ImpactDistance is a proximity-fuse-like radius.
sim = Simulation(round, target, env, [0 5], 0.001);
sim.Frame = 'FDR';
sim.Termination.ImpactDistance = 3.0;
sim.Termination.MaxDownrange   = 100.0;
sim.Visualizer = LiveEngagementPlotter(5); % finer decimation - engagement lasts ~1s

fprintf('Firing 35mm round...\n');
tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.3f s simulated).\n', toc, telemetry.time(end));

% 7. Post-Engagement Analysis
final_pos = telemetry.pos(end, :)';
final_tgt = telemetry.target_pos(end, :)';
miss_dist = norm(final_pos - final_tgt);

fprintf('Termination Reason  : %s\n', sim.TerminationReason);
fprintf('Final Miss Distance : %.2f m\n', miss_dist);

plotTrajectoryProfile(telemetry, final_tgt, '35mm Anti-Drone Engagement');
