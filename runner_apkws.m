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

% 1. Setup Environment
env = Environment(); % No wind, standard atmosphere

% 2. Setup Inbound Target (same as runner_sam_engagement.m)
% target = TargetConstantVelocity([8000; 0; 3000], [-250; 0; 0]);
target = TargetStatic([8000; 0; 0], [], 'NED');

% 3. Instantiate Guided Interceptor via MunitionFactory
missile = MunitionFactory.createGuidedRocketDatcom('MISSILEDATCOM_HYDRA70APKWS_motorburnout_canardsdeployed');

% 4. Initial Launch State: simple direct-aim at the target's current
%    position (no fire-control lead solution). The seeker/guidance loop
%    is what actually closes the intercept once it uncages at T_SeekerOn.
launcher_pos = [0; 0; 0];
los_vec      = target.Position - launcher_pos;
% launch_elev  = atan2(los_vec(3), los_vec(1));
launch_elev = pi/4; % positive makes sense
v0_rail = 60.0; % m/s, rail-launch speed before motor ignition, magnitude

missile.Dynamics.InitialState = missile.buildLaunchState(launcher_pos, v0_rail, launch_elev, 0);

fprintf('Launch elevation: %.1f mils\n', FireControlComputer.rad2mil(launch_elev));

% 5. Assemble Simulation & Execute
sim = Simulation(missile, target, env, [0 60], 0.005);
% sim.Termination.ImpactDistance = 3.0;
% sim.Termination.MaxDownrange   = 500.0;
sim.Termination.MinAltitude = 0;
% sim.Termination.MaxTime = 20;
sim.Termination.MaxTime = 2;
sim.Visualizer = LiveEngagementPlotter(20);
sim.Frame = 'NED';
sim.Visualizer.Frame = 'NED';
sim.TimeSpan = [0 100];
fprintf('Starting engagement simulation...\n');
tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.2f s simulated).\n', toc, telemetry.time(end));

% 6. Post-Engagement Analysis
final_pos = telemetry.pos(end, :)';
final_tgt = telemetry.target_pos(end, :)';
miss_dist = norm(final_pos - final_tgt);

fprintf('Termination Reason   : %s\n', sim.TerminationReason);
fprintf('Final Miss Distance  : %.2f m\n', miss_dist);

plotTrajectoryProfile(telemetry, final_tgt, 'APKWS Engagement');
