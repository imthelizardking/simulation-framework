%% Scenario: 155mm M107 Direct Fire (User-Specified Launch Conditions)
% Direct-fire counterpart to runner_m107.m: instead of FireControlComputer
% solving for the elevation/azimuth needed to hit a target, you specify
% the firing conditions yourself and see where the round goes.
% fcc.solveDirect(...) wraps them in the same sol-shaped struct
% solveNumericalShooting/solveFiringAngles return, so everything from
% buildLaunchState onward is identical to the indirect-fire runners.
clear; clc; close all;

charge_name   = 'Charge7'; % edit: 'Charge5' | 'Charge6' | 'Charge7'
elevation_deg = 35.0;      % edit: quadrant elevation
azimuth_deg   = 0.0;       % edit: aim bearing

% 1. Setup Environment
env = Environment(1.225, 9.81, [0; 0; 0]);

% 2. Instantiate Vehicle via MunitionFactory
shell = MunitionFactory.createM107Artillery(charge_name);
v0 = shell.Charge.MuzzleVelocity;
launcher_pos = [0; 0; 0];

% 3. Fire Control: direct fire - no solving, just wraps the given angles
fcc = FireControlComputer(env.Gravity, 'Z-Up');
sol = fcc.solveDirect(deg2rad(elevation_deg), deg2rad(azimuth_deg));

if ~sol.IsValid
    error('Elevation/azimuth outside the fire control computer''s mechanical limits.');
end

fprintf('Firing M107 on %s charge (%.0f m/s) at QE=%.1f deg, azimuth=%.1f deg...\n', ...
    charge_name, v0, elevation_deg, azimuth_deg);

% 4. Apply Specified Initial State
shell.Dynamics.InitialState = shell.buildLaunchState(launcher_pos, v0, sol.Elevation, sol.Azimuth);

% 5. Assemble Simulation & Execute. No target - default Termination
% (ground impact via MinAltitude) is exactly what direct-fire trajectory
% generation needs. TimeSpan is a generous fixed span (no sol.TimeOfFlight
% to size it from - direct fire doesn't solve for a target intercept).
sim = Simulation(shell, [], env, [0 90], 0.005);

tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.2f s simulated).\n', toc, telemetry.time(end));

% 6. Post-Flight Analysis
[apogee, apogee_idx] = max(telemetry.pos(:,3));
impact_range = telemetry.pos(end, 1);

fprintf('Termination Reason : %s\n', sim.TerminationReason);
fprintf('Apogee              : %.1f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Impact Range        : %.1f m\n', impact_range);
fprintf('Time of Flight      : %.2f s\n', telemetry.time(end));

plotTrajectoryProfile(telemetry, [], sprintf('M107 Direct Fire QE=%.0f deg (%s)', elevation_deg, charge_name));
