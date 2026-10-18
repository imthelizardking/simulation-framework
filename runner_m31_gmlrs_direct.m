%% Scenario: M31 GMLRS Unitary - Direct Fire (Dynamics-Only, No Guidance)
% Strips runner_m31_gmlrs_himars.m down to just the raw 6-DoF flight: no
% target, no GNSS/IMU/SAL sensors, no GMLRSMidcourseGuidance, no
% Autopilot6DoF - MunitionFactory.createM31GMLRSUnitary already leaves
% Sensors/Guidance/Controller at {}/[]/[], so fin deflection stays 0 for
% the whole flight (Munition.getDerivatives' own no-Controller default).
% This is exactly the round's open-loop Dynamics6DoF/AeroMissile6DoF/
% Propulsion_SolidMotor physics under a fixed elevation/azimuth, nothing
% steering it - useful for checking the raw dynamics (boost-phase
% acceleration, coast trajectory, stability) in isolation before layering
% the guidance loop back on.
clear; clc; clf;

elevation_mil = 800.0; % edit: quadrant elevation (NATO mils, 6400/circle)
azimuth_mil   = 0.0;   % edit: aim bearing
v0_rail       = 50.0;  % m/s - rail-launch speed (same illustrative value runner_m31_gmlrs_himars.m uses; the real boost comes from Propulsion_SolidMotor after this)

% 1. Setup Environment
env = Environment();
launcher_pos = [0; 0; 0];

% 2. Instantiate the round
missile = MunitionFactory.createM31GMLRSUnitary();

% 3. Initial Launch State (direct fire - fixed elevation/azimuth, no solve)
missile.Dynamics.InitialState = missile.buildLaunchState(launcher_pos, v0_rail, ...
    FireControlComputer.mil2rad(elevation_mil), FireControlComputer.mil2rad(azimuth_mil));

% 4. Assemble Simulation & Execute. No target - default Termination
% (ground impact via MinAltitude) is exactly what a dynamics-check flight
% needs, same as runner_m107_direct.m. TimeSpan is a generous fixed span
% (no solved TimeOfFlight to size it from - direct fire doesn't solve for
% an intercept); a bit longer than the pure-ballistic artillery runners'
% own 90s since GMLRS's rocket boost can loft it further.
sim = Simulation(missile, [], env, [0 120], 0.005);
sim.Frame = 'NED'; % see Dynamics6DoF's class comment - do not change to 'Z-Up'

fprintf('Firing M31 GMLRS Unitary (direct fire, no guidance) at %.1f mils elevation, v0 = %.0f m/s...\n', ...
    elevation_mil, v0_rail);
tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.2f s simulated).\n', toc, telemetry.time(end));

% 5. Post-Flight Analysis
[apogee, apogee_idx] = max(CoordinateFrame.altitudeColumn(telemetry.pos(:,3), telemetry.Frame));
final_pos    = telemetry.pos(end, :)';
downrange    = final_pos(1);
deflection   = final_pos(2);
impact_range = hypot(downrange, deflection);

fprintf('Termination Reason : %s\n', sim.TerminationReason);
fprintf('Apogee              : %.1f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Downrange           : %.1f m\n', downrange);
fprintf('Deflection          : %.1f m\n', deflection);
fprintf('Impact Range        : %.1f m\n', impact_range);
fprintf('Time of Flight      : %.2f s\n', telemetry.time(end));

plotTrajectoryProfile(telemetry, [], 'M31 GMLRS Unitary - Direct Fire (Dynamics Check)');
