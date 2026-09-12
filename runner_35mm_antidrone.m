%% Scenario: 35mm Anti-Drone Direct-Fire Engagement (Land-to-Air)
clear; clc; close all;

% 1. Setup Environment (standard atmosphere, no wind)
env = Environment();

% 2. Setup Target: small commercial quadcopter (DJI-class), level flight
%    at a typical cruise speed (~14 m/s here), crossing/closing on the gun
%    position at a modest patrol altitude.
target = TargetConstantVelocity([1000; 200; 120], [-12; 8; 0]);

% 3. Instantiate Munition via MunitionFactory
round = MunitionFactory.create35mmHE();

% 4. Fire Control: analytical direct-lead solve. Unlike
%    solveNumericalShooting (built for a static target), solveDirectLead
%    accounts for the target's velocity - the right tool for a direct-fire
%    gun engagement against something moving, rather than a lobbed shot at
%    a fixed point.
fcc = FireControlComputer(env.Gravity, 'Z-Up');
v0 = round.Charge.MuzzleVelocity; % see MunitionFactory.create35mmHE
launcher_pos = [0; 0; 0];

sol = fcc.solveFiringAngles(launcher_pos, v0, target.getTargetBus(), 'DirectLead');

if ~sol.IsValid
    error('No valid direct-lead solution (target out of envelope or geometry infeasible).');
end

fprintf('=== Direct-Lead Solution ===\n');
fprintf('Elevation                : %.1f mils\n', sol.Elevation);
fprintf('Azimuth                  : %.1f mils\n', sol.Azimuth);
fprintf('Predicted Intercept Time : %.3f s\n', sol.TimeOfFlight);

% 5. Apply Solved Initial State
round.Dynamics.InitialState = round.buildLaunchState(launcher_pos, v0, ...
    FireControlComputer.mil2rad(sol.Elevation), FireControlComputer.mil2rad(sol.Azimuth));

% 6. Assemble Simulation & Execute. Unguided direct-fire round with no
%    terminal correction - ImpactDistance is set to a proximity-fuse-like
%    radius rather than a tight direct-hit tolerance, because
%    solveDirectLead's quadratic assumes constant bullet velocity and
%    ignores drag: the real round decelerates in flight (CD0=0.25 at
%    ~Mach 3.4 is not negligible), so it arrives slightly later than
%    predicted and the lead point is imperfect by construction.
sim = Simulation(round, target, env, [0 5], 0.001);
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
