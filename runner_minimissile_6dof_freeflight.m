%% Scenario: Mini Missile - Genuine 6-DoF Free-Flight Physics Verification
% No guidance/controller yet - this is the free-flight physics core
% (Dynamics6DoF + AeroMissile6DoF) on its own, meant to exercise the new
% real rigid-body attitude/moment physics in its actual Simulation context
% before any autopilot is layered on top. See MunitionFactory.
% createMiniMissile6DoF and AeroMissile6DoF's own comments for the current
% state of the coefficient set (damping/control derivatives default to 0
% until real Missile DATCOM $DAMP/$CONTROL data is supplied).
%
% Expected behavior with Cm_q=Cn_r=Cl_p=0 (no rate damping yet): the
% airframe is still statically stable (Cm_alpha, Cn_beta < 0), so any
% disturbance produces a persistent, UNDAMPED oscillation in alpha/beta
% rather than either diverging or settling - that's the physically correct
% signature of "real static stability, zero damping," not a bug. Don't
% expect a clean, settled flight until real damping derivatives are wired
% in.
clear; clc; clf;

launch_angle_mil = 800.0;

% 1. Setup Atmosphere
env = Atmosphere();

% 2. Instantiate Vehicle
missile = MunitionFactory.createMiniMissile6DoF();
v0 = 50.0; % m/s - modest rail-launch speed; the solid motor takes over from there

% 3. Initial Launch State - elevation/azimuth in the familiar Z-Up sense
% (see Dynamics6DoF.buildLaunchState); IMPORTANT: sim.Frame below MUST be
% 'NED' - see Dynamics6DoF's own class comment on why.
launcher_pos = [0; 0; 0];
launch_angle = FireControlComputer.mil2rad(launch_angle_mil);
missile.Dynamics.InitialState = missile.buildLaunchState(launcher_pos, v0, launch_angle, 0);

% 4. Assemble Simulation & Execute
sim = Simulation(missile, [], env, [0 30], 0.001);
sim.Frame = 'NED'; % see Dynamics6DoF's class comment - do not change to 'Z-Up'

fprintf('Launching mini missile at %.1f mils elevation, v0 = %.0f m/s...\n', launch_angle_mil, v0);
tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.2f s simulated).\n', toc, telemetry.time(end));

% 5. Post-Flight Analysis - telemetry only carries pos/vel/a mislabeled
% "pitch" scalar (Simulation's generic telemetry predates 17-state
% vehicles - see its own tel.pitch(k)=X(7) line, which is roll (phi) for
% this class, not pitch), so attitude/rate diagnostics come from the
% final integrated state directly instead. Attitude comes from the
% quaternion (Dynamics6DoF.IDX_QUAT) - IDX_EULER is only a one-time
% launch snapshot for this class now, not live (see Dynamics6DoF's
% header comment on why it switched to quaternion attitude).
X_final = missile.Dynamics.CurrentState;
euler_f = Dynamics6DoF.quatToEuler(X_final(Dynamics6DoF.IDX_QUAT));
phi_f = euler_f(1); theta_f = euler_f(2); psi_f = euler_f(3);
p_f = X_final(BaseDynamics.IDX_BODYRATE(1));
q_f = X_final(BaseDynamics.IDX_BODYRATE(2));
r_f = X_final(BaseDynamics.IDX_BODYRATE(3));

R_bi_f = Dynamics6DoF.quatToDCM(X_final(Dynamics6DoF.IDX_QUAT));
v_body_f = R_bi_f.' * X_final(BaseDynamics.IDX_VEL);
alpha_f = atan2(v_body_f(3), v_body_f(1));
beta_f  = asin(max(-1, min(1, v_body_f(2) / norm(v_body_f))));

[apogee, apogee_idx] = max(CoordinateFrame.altitudeColumn(telemetry.pos(:,3), 'NED'));
final_pos = telemetry.pos(end, :)';
ground_range = hypot(final_pos(1), final_pos(2)); % NED: X=forward, Y=right

fprintf('\n=== Results ===\n');
fprintf('Termination Reason  : %s\n', sim.TerminationReason);
fprintf('Apogee              : %.1f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Ground Range        : %.1f m\n', ground_range);
fprintf('Time of Flight      : %.2f s\n', telemetry.time(end));
fprintf('Final Euler (deg)   : phi=%.2f theta=%.2f psi=%.2f\n', ...
    rad2deg(phi_f), rad2deg(theta_f), rad2deg(psi_f));
fprintf('Final Body Rates    : p=%.3f q=%.3f r=%.3f rad/s\n', p_f, q_f, r_f);
fprintf('Final Alpha/Beta    : alpha=%.2f deg, beta=%.2f deg\n', rad2deg(alpha_f), rad2deg(beta_f));
fprintf('Final Speed         : %.1f m/s\n', norm(X_final(BaseDynamics.IDX_VEL)));

if any(isnan(X_final)) || any(isinf(X_final))
    warning('Final state contains NaN/Inf - the physics diverged. Check MassProperties.Inertia and RefArea/RefLength are physically consistent before trusting this model further.');
end

plotTrajectoryProfile(telemetry, [], sprintf('Mini Missile 6-DoF Free Flight @ %.0f mils', launch_angle_mil));
