%% Scenario: 155mm M107 Direct Fire under METCM Atmospheric Data
% Same direct-fire setup as runner_m107_direct.m, but the Environment's
% atmosphere comes from data/metcm_std.txt (see Environment.loadMetcm)
% instead of the built-in ISA formulas. metcm_std.txt represents "standard
% conditions" (calm, near-ISA), so this is really a sanity check that the
% METCM path produces trajectories close to the ISA-based runner, not a
% scenario built to show a big difference - swap in a real (non-standard)
% METCM file to actually see one.
clear; clc; close all;

charge_name   = 'Charge7'; % edit: 'Charge5' | 'Charge6' | 'Charge7'
elevation_mil = 600;       % edit: quadrant elevation (NATO mils, 6400/circle)
azimuth_mil   = 0.0;       % edit: aim bearing

% 1. Setup Environment: ISA by default, then switch to the METCM profile.
env = Environment(1.225, 9.81, [0; 0; 0]);
env.loadMetcm('metcm_std');

% Quick sanity check against plain ISA at a few altitudes this trajectory
% will actually pass through - should be close, since the file is a calm,
% near-standard atmosphere.
env_isa = Environment(1.225, 9.81, [0; 0; 0]); % no loadMetcm - stays ISA
fprintf('%8s %12s %12s %12s %12s\n', 'Alt(m)', 'ISA T(K)', 'METCM T(K)', 'ISA rho', 'METCM rho');
for h = [0 1000 2000 3000 4000]
    fprintf('%8d %12.2f %12.2f %12.4f %12.4f\n', h, ...
        env_isa.getTemperature(h), env.getTemperature(h), ...
        env_isa.getDensity(h), env.getDensity(h));
end
fprintf('\n');

% 2. Instantiate Vehicle via MunitionFactory
shell = MunitionFactory.createM107Artillery(charge_name);
v0 = shell.Charge.MuzzleVelocity;
launcher_pos = [0; 0; 0];

% 3. Fire Control: direct fire - no solving, just wraps the given angles
fcc = FireControlComputer(env.Gravity, 'Z-Up');
sol = fcc.solveDirect(FireControlComputer.mil2rad(elevation_mil), FireControlComputer.mil2rad(azimuth_mil));

if ~sol.IsValid
    error('Elevation/azimuth outside the fire control computer''s mechanical limits.');
end

fprintf('Firing M107 on %s charge (%.0f m/s) at QE=%.1f mils under METCM atmosphere...\n', ...
    charge_name, v0, elevation_mil);

% 4. Apply Specified Initial State
shell.Dynamics.InitialState = shell.buildLaunchState(launcher_pos, v0, sol.Elevation, sol.Azimuth);

% 5. Assemble Simulation & Execute
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

plotTrajectoryProfile(telemetry, [], sprintf('M107 Direct Fire QE=%.0f mils (METCM atmosphere)', elevation_mil));
