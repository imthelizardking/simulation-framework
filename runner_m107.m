%% Master Scenario: 155mm M107 Artillery via MunitionFactory
clear; clc; close all;

% 1. Setup Environment (Standard atmosphere, optional crosswind)
env = Environment(); % No wind

% 2. Setup Target
target = TargetStatic([7500; 350; 0]); % Target 7.5 km downrange, 350 m offset

% 3. Instantiate Vehicle via MunitionFactory (Replaces all manual assembly!)
shell = MunitionFactory.createM107Artillery();

% 4. Fire Control Computer (Solves Quadrant Elevation & Wind Crab Angle)
fcc = FireControlComputer(env.Gravity, 'Z-Up');
v0_charge7 = shell.Charge.MuzzleVelocity; % M119A2 Charge7, see MunitionFactory.createM107Artillery
launcher_pos = [0; 0; 0];

fprintf('Running Fire Control numerical shooting solver...\n');
sol = fcc.solveNumericalShooting(shell, launcher_pos, v0_charge7, target.getTargetBus(), env, 'Low');

if ~sol.IsValid
    error('Target is out of range or solution failed to converge!');
end

fprintf('=== Solution Converged in %d Iterations ===\n', sol.Iterations);
fprintf('Quadrant Elevation (QE) : %.1f mils\n', sol.Elevation);
fprintf('Azimuth (Aim Bearing)   : %.1f mils (Wind corrected)\n', sol.Azimuth);
fprintf('Predicted Flight Time   : %.2f s\n', sol.TimeOfFlight);
fprintf('Convergence Miss Error  : %.3f m\n', sol.MissDistance);

% 5. Apply Solved Initial State to Munition
shell.Dynamics.InitialState = shell.buildLaunchState(launcher_pos, v0_charge7, ...
    FireControlComputer.mil2rad(sol.Elevation), FireControlComputer.mil2rad(sol.Azimuth));

% 6. Assemble Simulation & Execute
% NOTE: no Visualizer attached - LiveEngagementPlotter works fine against
% an unguided ballistic shell too (it degrades its seeker/FOV and command-
% breakdown panels gracefully when absent), but a post-flight static plot
% is still the better fit for a pure trajectory-generation scenario like
% this one - there's no live engagement decision-making to watch unfold.
sim = Simulation(shell, target, env, [0 sol.TimeOfFlight * 1.05], 0.005);
sim.Frame = 'NED';

fprintf('Starting trajectory integration...\n');
tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.2f s simulated).\n', toc, telemetry.time(end));

% 7. Post-Flight Analysis
final_pos = telemetry.pos(end, :)';
actual_miss = norm(final_pos(1:2) - target.Position(1:2));
[apogee, apogee_idx] = max(CoordinateFrame.altitudeColumn(telemetry.pos(:,3), telemetry.Frame));

fprintf('Termination Reason          : %s\n', sim.TerminationReason);
fprintf('Apogee                      : %.1f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Actual Impact Miss Distance : %.2f meters\n', actual_miss);
fprintf('Forward Range               : %.0f m\n', final_pos(1));
fprintf('Deflection                  : %.0f m\n', final_pos(2));
fprintf('Total Range                 : %.0f m\n', hypot(final_pos(1), final_pos(2)));

plotTrajectoryProfile(telemetry, target.Position, 'M107 Artillery Trajectory');
