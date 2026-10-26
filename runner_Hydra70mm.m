%% Scenario: Guided Rocket with Missile DATCOM Aero Tables vs. Inbound Target
% Same engagement shape as runner_sam_engagement.m, but the interceptor's
% Aero comes from Aero6DoF (via ParserDatcom), reading Missile DATCOM's
% own printed CD/CL-vs-alpha tables directly instead of the CD0/CL_alpha
% buildup APKWS uses. Defaults to data/ex1_in.txt, a real Digital DATCOM
% run (the DATCOM Users Manual's "Example Problem 1" body) - swap in your
% own rocket's DATCOM run once you have one:
%   MunitionFactory.createGuidedRocketDatcom('your_datcom_output_name')
% (data/<name>.txt must be DATCOM's own printed output listing, not a
% MATLAB script - see ParserDatcom.)
clear; clc; clf;

% 1. Setup Atmosphere
env = Atmosphere(); % No wind, standard atmosphere

% 2. Setup Inbound Target (same as runner_sam_engagement.m)
% target = TargetConstantVelocity([8000; 0; 3000], [-250; 0; 0]);

% 3. Instantiate Guided Interceptor via MunitionFactory
missile = MunitionFactory.createHydra70mm('MISSILEDATCOM_HYDRA70');

% 4. Initial Launch State: simple direct-aim at the target's current
%    position (no fire-control lead solution). The seeker/guidance loop
%    is what actually closes the intercept once it uncages at T_SeekerOn.
launcher_pos = [0; 0; 0];
launch_elev = pi/4; % positive makes sense
v0_rail = 45.0; % m/s, rail-launch speed before motor ignition, magnitude

missile.Dynamics.InitialState = missile.buildLaunchState(launcher_pos, v0_rail, launch_elev, 0);

fprintf('Launch elevation: %.1f mils\n', FireControlComputer.rad2mil(launch_elev));

% 5. Assemble Simulation & Execute
sim = Simulation(missile, [], env, [0 60], 0.1);
% sim.Termination.MaxTime = 1.5;
sim.Termination.MinAltitude = 0;
sim.Visualizer = Visualizer_Basic(20,[0, 20000, 0, 6000]);
sim.Visualizer.Frame = 'NED';
sim.Frame = 'NED';
sim.TimeSpan = [0 100];
fprintf('Starting simulation...\n');
tic;
telemetry = sim.run();
fprintf('Computational time spent is %.2f seconds (%.2f s flight simulated).\n', toc, telemetry.time(end));

% 6. Post-Engagement Analysis
final_pos = telemetry.pos(end, :)';

fprintf('Termination Reason   : %s\n', sim.TerminationReason);

plotTrajectoryProfile(telemetry, [], 'Hydra70mm traj');
