%% Master Scenario: 155mm M107 Artillery (MPMM High-Fidelity Model)
% Same scenario as before, now built directly on the high-fidelity MPMM
% pair (DynamicsMPMM + AeroMPMM - see those files) instead of
% MunitionFactory.createM107Artillery's simpler Dynamics3DofSpin +
% AeroTabulatedSpin model, now that the MPMM physics has been verified
% (see runner_mpmm_test.m for a side-by-side comparison against the
% simpler model).
clear; clc; close all;

% 1. Setup Environment (Standard atmosphere, optional crosswind)
env = Environment(); % No wind
launcher_pos = [0; 0; 0];
latitude_deg = 39.0; % edit: launch latitude - feeds real gravity/Coriolis

% 2. Setup Target
target = TargetStatic([7500; 350; 0]); % Target 7.5 km downrange, 350 m offset

% 3. Instantiate Vehicle: same mass/inertia/reference dims/CD0/CLA/CSPIN/
%    CMAG/CMA/twist as MunitionFactory.createM107Artillery - see that
%    preset for where these illustrative values come from.
shell = Munition('M107_155mm');
shell.Type = 'HE';
shell.MassProperties.Mass = 43.091;
shell.MassProperties.Inertia = diag([0.13, 1.62, 1.62]);
shell.TwistRate = 20;
shell.Charge = ChargeFactory.getM119A2('Charge7');

shell.Dynamics = DynamicsMPMM();
shell.Dynamics.InitLatitudeDeg = latitude_deg;
shell.Dynamics.SimCoriolis     = true;

aero = AeroMPMM();
aero.RefArea     = 0.01887;
aero.RefLength   = 0.155;
aero.Ix          = shell.MassProperties.Inertia(1,1);
aero.LatitudeDeg = latitude_deg;
aero.CD0   = 0.22;
aero.CLA   = 2.5;
aero.CSPIN = -0.03;
aero.CMA   = 2.0;
aero.CMAG  = 1.2;
% Form factors come from the charge itself (see ChargeZone.IForm/FL/QD/
% QM, populated from data/getM119A2_Charge7.<ext> if present - see
% ChargeFactory.applyFormFactors), not hardcoded here.
aero.IForm = shell.Charge.IForm;
aero.FL    = shell.Charge.FL;
aero.QD    = shell.Charge.QD;
aero.QM    = shell.Charge.QM;
shell.Aero = aero;

% 4. Fire Control Computer (Solves Quadrant Elevation & Wind Crab Angle)
fcc = FireControlComputer(env.Gravity, 'Z-Up');
v0_charge7 = shell.Charge.MuzzleVelocity; % M119A2 Charge7

fprintf('Running Fire Control numerical shooting solver...\n');
sol = fcc.solveNumericalShooting(shell, launcher_pos, v0_charge7, target.getTargetBus(), env, 'Low');

if ~sol.IsValid
    error('Target is out of range or solution failed to converge!');
end

fprintf('=== Solution Converged in %d Iterations ===\n', sol.Iterations);
fprintf('Quadrant Elevation (QE) : %.1f mils\n', sol.Elevation);
fprintf('Azimuth (Aim Bearing)   : %.1f mils (Wind corrected)\n', sol.Azimuth);
fprintf('Predicted Flight Time   : %.2f s\n', sol.TimeOfFlight);
fprintf('Convergence Miss Error  : %.3f m\n', sol.MissDistance);

% 5. Apply Solved Initial State to Munition
shell.Dynamics.InitialState = shell.buildLaunchState(launcher_pos, v0_charge7, ...
    FireControlComputer.mil2rad(sol.Elevation), FireControlComputer.mil2rad(sol.Azimuth));

% 6. Assemble Simulation & Execute
% NOTE: no Visualizer attached - LiveEngagementPlotter works fine against
% an unguided ballistic shell too (it degrades its seeker/FOV and command-
% breakdown panels gracefully when absent), but a post-flight static plot
% is still the better fit for a pure trajectory-generation scenario like
% this one - there's no live engagement decision-making to watch unfold.
sim = Simulation(shell, target, env, [0 sol.TimeOfFlight * 1.05], 0.005);
sim.Frame = 'NED';

fprintf('Starting trajectory integration...\n');
tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.2f s simulated).\n', toc, telemetry.time(end));

% 7. Post-Flight Analysis
final_pos = telemetry.pos(end, :)';
actual_miss = norm(final_pos(1:2) - target.Position(1:2));
[apogee, apogee_idx] = max(CoordinateFrame.altitudeColumn(telemetry.pos(:,3), telemetry.Frame));

fprintf('Termination Reason          : %s\n', sim.TerminationReason);
fprintf('Apogee                      : %.1f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Actual Impact Miss Distance : %.2f meters\n', actual_miss);
fprintf('Forward Range               : %.0f m\n', final_pos(1));
fprintf('Deflection                  : %.0f m\n', final_pos(2));
fprintf('Total Range                 : %.0f m\n', hypot(final_pos(1), final_pos(2)));

plotTrajectoryProfile(telemetry, target.Position, 'M107 Artillery Trajectory (MPMM)');
