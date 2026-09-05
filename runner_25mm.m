%% Scenario: Basic 25mm M793 Ballistic Launch at a Fixed Angle
clear; clc; close all;

launch_angle_deg = 45.0; % edit this to try other quadrant elevations

% 1. Setup Environment (no wind - isolate the effect of launch angle alone)
env = Environment(1.225, 9.81, [0; 0; 0]);

% 2. Instantiate Vehicle via MunitionFactory
round = MunitionFactory.create25mmM793();
v0 = round.Charge.MuzzleVelocity; % see MunitionFactory.create25mmM793

% 3. Initial Launch State (Z-Up frame, planar X-Z launch)
launcher_pos = [0; 0; 0];
launch_angle = deg2rad(launch_angle_deg);
round.Dynamics.InitialState = round.Dynamics.buildLaunchState(...
    launcher_pos, v0, launch_angle, 0, round.MassProperties.Mass);

% 4. Assemble Simulation & Execute
% No target for this scenario - default Termination (ground impact via
% MinAltitude) is exactly what a pure ballistic launch needs.
sim = Simulation(round, [], env, [0 60], 0.001);

fprintf('Firing 25mm M793 at %.1f deg elevation, v0 = %.0f m/s...\n', launch_angle_deg, v0);
tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.2f s simulated).\n', toc, telemetry.time(end));

% 5. Post-Flight Analysis
[apogee, apogee_idx] = max(telemetry.pos(:,3));
impact_range = telemetry.pos(end, 1);

fprintf('Termination Reason : %s\n', sim.TerminationReason);
fprintf('Apogee              : %.1f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Impact Range        : %.1f m\n', impact_range);
fprintf('Time of Flight      : %.2f s\n', telemetry.time(end));

plotTrajectoryProfile(telemetry, [], sprintf('25mm M793 @ %.0f deg', launch_angle_deg));
