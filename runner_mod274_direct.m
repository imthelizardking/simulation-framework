%% Scenario: MOD274 155mm Direct Fire (User-Specified Launch Conditions)
% Direct-fire counterpart to runner_mod274.m: instead of solving for the
% elevation needed to hit a target, you specify firing conditions
% yourself - useful here in particular to watch the base-bleed drag
% reduction unfold over a known, fixed trajectory shape rather than
% whatever shape FireControlComputer's shooting solver happens to land on.
clear; clc; close all;

charge_name   = '6M';  % edit: '3M' | '4M' | '5M' | '6M'
elevation_mil = 711.1; % edit: quadrant elevation (NATO mils, near max-range for 6M)
azimuth_mil   = 0.0;   % edit: aim bearing

% 1. Setup Environment
env = Environment();

% 2. Instantiate Vehicle via MunitionFactory
shell = MunitionFactory.createMOD274(charge_name);
v0 = shell.Charge.MuzzleVelocity;
launcher_pos = [0; 0; 0];

% 3. Fire Control: direct fire - no solving, just wraps the given angles
fcc = FireControlComputer(env.Gravity, 'Z-Up');
sol = fcc.solveDirect(elevation_mil, azimuth_mil);

if ~sol.IsValid
    error('Elevation/azimuth outside the fire control computer''s mechanical limits.');
end

fprintf('Firing MOD274 on DM92 %s charge (%.0f m/s) at QE=%.1f mils, azimuth=%.1f mils...\n', ...
    charge_name, v0, elevation_mil, azimuth_mil);

% 4. Apply Specified Initial State
shell.Dynamics.InitialState = shell.buildLaunchState(launcher_pos, v0, ...
    FireControlComputer.mil2rad(sol.Elevation), FireControlComputer.mil2rad(sol.Azimuth));

% 5. Assemble Simulation & Execute
sim = Simulation(shell, [], env, [0 90], 0.005);

tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.2f s simulated).\n', toc, telemetry.time(end));

% 6. Post-Flight Analysis
[apogee, apogee_idx] = max(telemetry.pos(:,3));
impact_range = telemetry.pos(end, 1);
bb = shell.Aero.BaseBleed;

fprintf('Termination Reason      : %s\n', sim.TerminationReason);
fprintf('Apogee                  : %.1f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Impact Range            : %.1f m\n', impact_range);
fprintf('Time of Flight          : %.2f s\n', telemetry.time(end));
fprintf('Base Bleed Active Until : %.1f s (%.0f%% CD0 reduction while burning)\n', ...
    bb.BurnDuration, bb.DragReductionFraction * 100);

plotTrajectoryProfile(telemetry, [], sprintf('MOD274 Direct Fire QE=%.0f mils (%s)', elevation_mil, charge_name));
