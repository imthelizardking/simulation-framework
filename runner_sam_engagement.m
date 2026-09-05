%% Scenario: Basic Surface-to-Air Missile Engagement (APKWS-class Interceptor)
clear; clc; close all;

% 1. Setup Environment
env = Environment(1.225, 9.81, [0; 0; 0]); % No wind, standard atmosphere

% 2. Setup Inbound Target (constant velocity, closing on the launch site)
target = ConstantVelocityTarget([8000; 0; 3000], [-250; 0; 0]);

% 3. Instantiate Guided Interceptor via MunitionFactory
missile = MunitionFactory.createAPKWS();

% 4. Initial Launch State: simple direct-aim at the target's current
%    position (no fire-control lead solution). The seeker/guidance loop
%    is what actually closes the intercept once it uncages at T_SeekerOn.
launcher_pos = [0; 0; 0];
los_vec      = target.Position - launcher_pos;
launch_elev  = atan2(los_vec(3), los_vec(1));
v0_rail      = 60.0; % m/s, rail-launch speed before motor ignition

missile.Dynamics.InitialState = missile.Dynamics.buildLaunchState(...
    launcher_pos, v0_rail, launch_elev, 0, missile.MassProperties.Mass);

fprintf('Launch elevation: %.1f deg\n', rad2deg(launch_elev));

% 5. Assemble Simulation & Execute. This is an intercept scenario, so on
%    top of the default ground-impact check, also stop on a hit
%    (ImpactDistance) or a clean flyby past the target (MaxDownrange).
sim = Simulation(missile, target, env, [0 30], 0.005);
sim.Termination.ImpactDistance = 3.0;
sim.Termination.MaxDownrange   = 50.0;
sim.Visualizer = LiveEngagementPlotter(20);

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

plotTrajectoryProfile(telemetry, final_tgt, 'SAM Engagement');
