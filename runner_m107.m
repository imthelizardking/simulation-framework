%% Master Scenario: 155mm M107 Artillery via MunitionFactory
clear; clc; close all;

% 1. Setup Environment (Standard atmosphere, optional crosswind)
env = Environment(1.225, 9.81, [0; 5.0; 0]); % 5 m/s crosswind in Y (East)

% 2. Setup Target
target = StaticTarget([11200; 350; 0]); % Target 14.2 km downrange, 350 m offset

% 3. Instantiate Vehicle via MunitionFactory (Replaces all manual assembly!)
shell = MunitionFactory.createM107Artillery();

% 4. Fire Control Computer (Solves Quadrant Elevation & Wind Crab Angle)
fcc = FireControlComputer(env.Gravity, 'Z-Up');
v0_charge7 = 827.0; % Top zone muzzle velocity (m/s)
launcher_pos = [0; 0; 0];

fprintf('Running Fire Control numerical shooting solver...\n');
sol = fcc.solveNumericalShooting(shell, launcher_pos, v0_charge7, target.getTargetBus(), env, 'Low');

if ~sol.IsValid
    error('Target is out of range or solution failed to converge!');
end

fprintf('=== Solution Converged in %d Iterations ===\n', sol.Iterations);
fprintf('Quadrant Elevation (QE) : %.2f deg\n', rad2deg(sol.Elevation));
fprintf('Azimuth (Aim Bearing)   : %.2f deg (Wind corrected)\n', rad2deg(sol.Azimuth));
fprintf('Predicted Flight Time   : %.2f s\n', sol.TimeOfFlight);
fprintf('Convergence Miss Error  : %.3f m\n', sol.MissDistance);

% 5. Apply Solved Initial State to Munition
v_x = v0_charge7 * cos(sol.Elevation) * cos(sol.Azimuth);
v_y = v0_charge7 * cos(sol.Elevation) * sin(sol.Azimuth);
v_z = v0_charge7 * sin(sol.Elevation); % Z-Up frame

shell.Dynamics.InitialState = [launcher_pos; v_x; v_y; v_z; sol.Elevation; shell.MassProperties.Mass];

% 6. Assemble Simulation & Execute
% NOTE: no Visualizer attached - LiveEngagementPlotter expects a seeker-
% guided munition (gnc_bus.Sensors.SeekerLOAL, PN/Angle/FOV command bus)
% and will error against an unguided ballistic shell with no Sensors. Post-
% flight static plots below are the appropriate fit for a trajectory-only
% scenario like this one.
sim = Simulation(shell, target, env, [0 sol.TimeOfFlight * 1.05], 0.005);

fprintf('Starting trajectory integration...\n');
tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.2f s simulated).\n', toc, telemetry.time(end));

% 7. Post-Flight Analysis
final_pos = telemetry.pos(end, :)';
actual_miss = norm(final_pos(1:2) - target.Position(1:2));
[apogee, apogee_idx] = max(telemetry.pos(:,3));

fprintf('Termination Reason          : %s\n', sim.TerminationReason);
fprintf('Apogee                      : %.1f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Actual Impact Miss Distance : %.2f meters\n', actual_miss);

plotTrajectoryProfile(telemetry, target.Position, 'M107 Artillery Trajectory');
