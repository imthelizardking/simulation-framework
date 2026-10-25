%% Scenario: M31 GMLRS Unitary fired from HIMARS - GNSS/IMU Midcourse + SAL Terminal
% Engages a moving ground target (light armored vehicle, constant
% velocity) that the round cannot track during midcourse - it flies a
% lofted ballistic arc toward a FIXED aimpoint (the target's position at
% launch time) using its own dead-reckoning INS (GnssSensor + SensorIMU, fused
% inside Guidance_GMLRSMidcourse - see that class for the real strapdown
% mechanization and why its drift is a genuine, not simulated-for-show,
% error mechanism), then a Seeker_SAL takes over once a ground/airborne
% LaserDesignator lights the target within line-of-sight and the round is
% close enough to uncage - correcting for however far the target has
% moved since the aimpoint was set.
%
% Range kept to 25 km (well under GMLRS's real ~70 km class).
% solveNumericalShooting's solve flights are real Simulation runs capped
% at 300 s each, so a longer lofted shot is possible but slower to solve.
%
% IMPORTANT - fin effectiveness and autopilot gains: AeroMissile6DoF.
% Cm_delta/Cn_delta default to 0 in MunitionFactory.createM31GMLRSUnitary
% (honest - no real DATCOM $CONTROL data exists). This runner sets
% illustrative, UNTUNED placeholders below, and builds a SEPARATE
% Autopilot6DoF instance with gains sized for THIS vehicle's much larger
% inertia (Iyy=260 kg*m^2 vs the mini missile's 0.9) - do not assume the
% mini-missile intercept demo's gains would transfer, they would not.
% Expect to retune once you see actual behavior, same as that demo did.
clear; clc; clf;

% 1. Setup Environment
env = Environment();
launcher_pos = [0; 0; 0];

% 2. Moving ground target - light armoured vehicle, constant velocity
% (Z-Up sense: X=downrange, Y=crossrange, Z=altitude=0, ground level)
target = TargetConstantVelocity([25000; 1500; 0], [4; -6; 0]); % ~7.2 m/s road speed, diagonal path
aimpoint_zup = target.Position; % fixed midcourse aimpoint = target's position AT LAUNCH TIME - it will have moved by intercept, which is exactly what the SAL terminal phase corrects for

% 3. Laser designator - a ground observer on a 15 m mast near the
% target's route (swap to a higher Position(3) for an airborne/drone
% designator instead - LaserDesignator.hasLineOfSight uses the real
% geometric horizon formula, so altitude genuinely changes effective
% range). Positioned in Z-Up sense here for readability, then rotated to
% NED (LaserDesignator's own working frame) via the same
% CoordinateFrame.rotateVector every other class in this framework uses.
designator_pos_zup = [24000; 1000; 15];
designator = LaserDesignator(CoordinateFrame.rotateVector(designator_pos_zup, 'NED'), 6000.0, 1688);

% 4. Instantiate the round
missile = MunitionFactory.createM31GMLRSUnitary();

missile.Aero.Cm_delta = -3.0; % illustrative fin control-effectiveness placeholder - see this file's header comment
missile.Aero.Cn_delta = -3.0;

missile.Sensors  = { GnssSensor(), SensorIMU(), Seeker_SAL(designator) };
guidance = Guidance_GMLRSMidcourse();
guidance.AimPoint = CoordinateFrame.rotateVector(aimpoint_zup, 'NED');
missile.Guidance = guidance;

autopilot = Autopilot6DoF();
% Gains re-derived for THIS vehicle (see this file's header comment) -
% same "size Kp so the full MaxAlphaCmd error maps to just under
% MaxDeflection given Cm_delta" approach as the mini-missile fix, just
% recomputed for Cm_delta=-3.0 instead of -0.8. Kd_rate raised to
% compensate for the much larger inertia's slower natural response.
autopilot.Kp_alpha = 4.0;
autopilot.Kd_rate  = 1.0;
missile.Controller = autopilot;

% 5. Fire Control Computer - high-angle (lofted) solve toward the FIXED
% aimpoint, using the round's actual rail-launch speed; the recon flights
% inside solveNumericalShooting exercise the same Propulsion_SolidMotor
% boost phase the real round has, since munition_prototype IS this round
% (flown as real Simulation runs with its GNC stack detached - the
% uncontrolled ballistic arc; midcourse/SAL guidance corrects from there).
fcc = FireControlComputer(env.Gravity, 'Z-Up');
aim_target = TargetStatic(aimpoint_zup);
v0_rail = 50.0;

fprintf('Running Fire Control numerical shooting solver (lofted/High branch)...\n');
sol = fcc.solveNumericalShooting(missile, launcher_pos, v0_rail, aim_target.getTargetBus(), env, 'High', 0.005, 'NED');

if ~sol.IsValid
    error('Aimpoint out of range or solution failed to converge!');
end
fprintf('Quadrant Elevation (QE) : %.1f mils\n', sol.Elevation);
fprintf('Azimuth                 : %.1f mils\n', sol.Azimuth);
fprintf('Predicted Flight Time   : %.2f s\n', sol.TimeOfFlight);

missile.Dynamics.InitialState = missile.buildLaunchState(launcher_pos, v0_rail, ...
    FireControlComputer.mil2rad(sol.Elevation), FireControlComputer.mil2rad(sol.Azimuth));

% 6. Assemble Simulation & Execute
sim = Simulation(missile, target, env, [0, sol.TimeOfFlight * 1.3], 0.005);
sim.Frame = 'NED'; % see Dynamics6DoF's class comment - do not change to 'Z-Up'
sim.Termination.ImpactDistance = 15.0;
sim.Termination.MaxDownrange   = 3000.0;
sim.Visualizer = LiveEngagementPlotter(20);

fprintf('Launching M31 GMLRS Unitary...\n');
tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.2f s simulated).\n', toc, telemetry.time(end));

% 7. Post-Engagement Analysis
final_pos = telemetry.pos(end, :)';
final_tgt = telemetry.target_pos(end, :)';
miss_dist = norm(final_pos - final_tgt);
sal_locked = missile.LastGNCBus.Sensors.SAL.IsLocked;
guid_phase = missile.LastGNCBus.Guidance.Phase;

fprintf('\n=== Results ===\n');
fprintf('Termination Reason   : %s\n', sim.TerminationReason);
fprintf('Final Guidance Phase : %s\n', guid_phase);
fprintf('SAL Locked at End    : %s\n', mat2str(sal_locked));
fprintf('Final Miss Distance  : %.2f m\n', miss_dist);
fprintf('Target Drift Since Launch (aimpoint vs actual) : %.1f m\n', norm(target.Position - aimpoint_zup));

plotTrajectoryProfile(telemetry, final_tgt, 'M31 GMLRS Unitary - HIMARS Engagement');
