%% Scenario: Mini Missile - Genuine 6-DoF Guided Intercept, Live Plotting
% Same shape as runner_sam_engagement.m (constant-velocity inbound target,
% LiveEngagementPlotter for live animation), but flown by the new 6-DoF
% pipeline (Dynamics6DoF + AeroMissile6DoF) with a genuinely 3D seeker/
% guidance/autopilot chain (Seeker_3D + Guidance_ProportionalNav3D + Autopilot6DoF)
% instead of runner_sam_engagement's planar Seeker_LOAL/
% Guidance_OptimalAngleFOV/Controller_LongitudinalFin trio, which hardcodes
% Dynamics3Dof's 8-state layout (truth_state(7) as "pitch") and a single
% vertical plane - both wrong for this vehicle. Target carries a lateral
% (Y) offset specifically to exercise real 3D engagement geometry the old
% planar pipeline could never have handled at all.
%
% IMPORTANT - fin effectiveness: AeroMissile6DoF.Cm_delta/Cn_delta default
% to 0 in MunitionFactory.createMiniMissile6DoF (honestly - no real
% Missile DATCOM $CONTROL data exists yet), which means the fins produce
% NO moment and the missile CANNOT steer. This runner overrides both to
% illustrative, UNTUNED nonzero placeholders below purely so this demo
% actually demonstrates guidance - replace with real DATCOM-derived
% control-effectiveness numbers once you have them, and expect to re-tune
% Autopilot6DoF's Kp_alpha/Kd_rate gains once the real numbers change the
% vehicle's actual control authority.
clear; clc; clf;

% 1. Setup Environment
env = Environment();

% 2. Setup Inbound Target (constant velocity, closing on the launch site,
% WITH a lateral offset - Z-Up sense: X=downrange, Y=crossrange, Z=up)
target = TargetConstantVelocity([4000; 500; 1500], [-200; 0; 0]);

% 3. Instantiate the 6-DoF missile
missile = MunitionFactory.createMiniMissile6DoF();

% Illustrative fin control-effectiveness placeholders - see this file's
% header comment. Signs matter: Cm_delta/Cn_delta must have the same sign
% convention Autopilot6DoF assumes (a positive commanded moment coefficient
% resolves to a positive fin deflection) - these are plausible-magnitude
% placeholders, not derived from any real airframe.
missile.Aero.Cm_delta = -0.8;
missile.Aero.Cn_delta = -0.8;

missile.Sensors    = { Seeker_3D() };
missile.Guidance   = Guidance_ProportionalNav3D();
missile.Controller = Autopilot6DoF();

% 4. Initial Launch State: simple direct-aim at the target's current
% position (no fire-control lead solution) - the seeker/guidance/autopilot
% loop is what actually closes the intercept from here. IMPORTANT:
% sim.Frame below MUST be 'NED' - see Dynamics6DoF's own class comment.
launcher_pos = [0; 0; 0];
los_vec      = target.Position - launcher_pos;
launch_elev  = atan2(los_vec(3), hypot(los_vec(1), los_vec(2)));
launch_az    = atan2(los_vec(2), los_vec(1));
v0_rail      = 50.0; % m/s, rail-launch speed before the solid motor ignites

missile.Dynamics.InitialState = missile.buildLaunchState(launcher_pos, v0_rail, launch_elev, launch_az);

fprintf('Launch elevation: %.1f mils, azimuth: %.1f mils\n', ...
    FireControlComputer.rad2mil(launch_elev), FireControlComputer.rad2mil(launch_az));

% 5. Assemble Simulation & Execute - stop on ground impact (default), a
% hit (ImpactDistance), or a clean flyby past the target (MaxDownrange).
sim = Simulation(missile, target, env, [0 30], 0.002);
sim.Frame = 'NED'; % see Dynamics6DoF's class comment - do not change to 'Z-Up'
sim.Termination.ImpactDistance = 5.0;
sim.Termination.MaxDownrange   = 500.0;
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

plotTrajectoryProfile(telemetry, final_tgt, 'Mini Missile 6-DoF Intercept');
