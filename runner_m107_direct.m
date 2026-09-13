%% Scenario: 155mm M107 Direct Fire (User-Specified Launch Conditions)
% Direct-fire counterpart to runner_m107.m: instead of FireControlComputer
% solving for the elevation/azimuth needed to hit a target, you specify
% the firing conditions yourself and see where the round goes.
% fcc.solveDirect(...) wraps them in the same sol-shaped struct
% solveNumericalShooting/solveFiringAngles return, so everything from
% buildLaunchState onward is identical to the indirect-fire runners.
clear; clc; close all;

charge_name   = 'Charge7'; % edit: 'Charge5' | 'Charge6' | 'Charge7'
elevation_mil = 600;     % edit: quadrant elevation (NATO mils, 6400/circle)
azimuth_mil   = 0.0;       % edit: aim bearing

% 1. Setup Environment
env = Environment();

% 2. Instantiate Vehicle via MunitionFactory
shell = MunitionFactory.createM107Artillery(charge_name);
shell.Dynamics.Frame = 'NED'; % must be set before buildLaunchState below - it reads this at build time
shell.Aero.Frame     = 'NED';
v0 = shell.Charge.MuzzleVelocity;
launcher_pos = [0; 0; 0];

% 3. Fire Control: direct fire - no solving, just wraps the given angles
fcc = FireControlComputer(env.Gravity, 'Z-Up');
sol = fcc.solveDirect(elevation_mil, azimuth_mil);

if ~sol.IsValid
    error('Elevation/azimuth outside the fire control computer''s mechanical limits.');
end

fprintf('Firing M107 on %s charge (%.0f m/s) at QE=%.1f mils, azimuth=%.1f mils...\n', ...
    charge_name, v0, elevation_mil, azimuth_mil);

% 4. Apply Specified Initial State
shell.Dynamics.InitialState = shell.buildLaunchState(launcher_pos, v0, ...
    FireControlComputer.mil2rad(sol.Elevation), FireControlComputer.mil2rad(sol.Azimuth));

% 5. Assemble Simulation & Execute. No target - default Termination
% (ground impact via MinAltitude) is exactly what direct-fire trajectory
% generation needs. TimeSpan is a generous fixed span (no sol.TimeOfFlight
% to size it from - direct fire doesn't solve for a target intercept).
sim = Simulation(shell, [], env, [0 90], 0.005);
sim.Frame = 'NED';

tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.2f s simulated).\n', toc, telemetry.time(end));

% 6. Post-Flight Analysis
[apogee, apogee_idx] = max(CoordinateFrame.altitudeColumn(telemetry.pos(:,3), telemetry.Frame));
forward_range = telemetry.pos(end, 1);
deflection    = telemetry.pos(end, 2);
total_range   = hypot(forward_range, deflection);

fprintf('Termination Reason : %s\n', sim.TerminationReason);
fprintf('Apogee              : %.1f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Forward Range       : %.0f m\n', forward_range);
fprintf('Deflection          : %.0f m\n', deflection);
fprintf('Total Range         : %.0f m\n', total_range);
fprintf('Time of Flight      : %.2f s\n', telemetry.time(end));

plotTrajectoryProfile(telemetry, [], sprintf('M107 Direct Fire QE=%.0f mils (%s)', elevation_mil, charge_name));
