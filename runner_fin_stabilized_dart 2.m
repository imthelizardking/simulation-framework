%% Scenario: Basic Fin-Stabilized Dart Toss (Plain 3DoF Longitudinal Model)
% The simplest possible scenario in this framework: no fire control
% solver, no target, no spin - just Dynamics3Dof + Aero3DoF
% acting on a hand-thrown fin-stabilized dart, to see the basic pitch-
% plane lift/drag/pitch-follower model in isolation. Contrast with
% runner_25mm.m and friends, which now use DynamicsMPMM/
% AeroMPMM for their rifled, spin-stabilized rounds.
clear; clc; clf;

launch_angle_mil = 622.2; % edit this to try other launch angles (NATO mils, 6400/circle)

% 1. Setup Atmosphere (no wind - isolate the effect of launch angle alone)
env = Atmosphere();

% 2. Instantiate Munition via MunitionFactory
dart = MunitionFactory.createFinStabilizedDart();
v0 = dart.Charge.MuzzleVelocity; % see MunitionFactory.createFinStabilizedDart

% 3. Initial Launch State (Z-Up frame, planar X-Z throw)
launcher_pos = [0; 0; 1.8]; % thrown from about head height
launch_angle = FireControlComputer.mil2rad(launch_angle_mil);
dart.Dynamics.InitialState = dart.buildLaunchState(launcher_pos, v0, launch_angle, 0);

% 4. Assemble Simulation & Execute
% No target for this scenario - default Termination (ground impact via
% MinAltitude) is exactly what a pure ballistic throw needs.
sim = Simulation(dart, [], env, [0 10], 0.001);

fprintf('Throwing fin-stabilized dart at %.1f mils, v0 = %.1f m/s...\n', launch_angle_mil, v0);
tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.2f s simulated).\n', toc, telemetry.time(end));

% 5. Post-Flight Analysis
[apogee, apogee_idx] = max(telemetry.pos(:,3));
throw_range = telemetry.pos(end, 1);

fprintf('Termination Reason : %s\n', sim.TerminationReason);
fprintf('Apogee              : %.2f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Throw Range         : %.2f m\n', throw_range);
fprintf('Time of Flight      : %.2f s\n', telemetry.time(end));

plotTrajectoryProfile(telemetry, [], sprintf('Fin-Stabilized Dart @ %.0f mils', launch_angle_mil));
