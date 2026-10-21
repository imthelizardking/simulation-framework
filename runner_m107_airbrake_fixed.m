%% Master Scenario: 155mm M107 with a Fixed (Manually-Set) Airbrake Deploy Time
% Exercises AeroMPMM's airbrake_ADT/Phase2Active one-shot coefficient
% switch in isolation - no sensor, no controller, no deploy-time solver.
% The deploy time is just a plain scalar set directly below. Otherwise
% identical to runner_m107.m (same fire-control solve, same launch).
clear; clc; clf;

airbrake_deploy_time = 12.0; % s since launch - edit this to see its effect on range

% 1. Setup Environment (Standard atmosphere, optional crosswind)
env = Environment(); % No wind
launcher_pos = [0; 0; 0];
latitude_deg = 39.0; % edit: launch latitude - feeds real gravity/Coriolis

% 2. Setup Target
target = TargetStatic([7500; 0; 350], [], 'FDR'); % forward=7.5km, down=0 (ground level), right=350m offset

% 3. Instantiate Vehicle
shell = MunitionFactory.createM107Artillery('CH7W');
shell.Dynamics.InitLatitudeDeg = latitude_deg;
shell.Dynamics.SimCoriolis     = true;
shell.Aero.LatitudeDeg         = latitude_deg;

% Airbrake deploy time - Phase2 coefficients themselves already come from
% MunitionFactory.createM107Artillery (data/M107_155mm.mat's aero.phase2.*
% section if present, otherwise an illustrative CD0 bump - see that
% factory method), so only the deploy time needs setting here.
shell.Aero.airbrake_ADT = airbrake_deploy_time;

% 4. Fire Control Computer (Solves Quadrant Elevation & Wind Crab Angle)
fcc = FireControlComputer(env.Gravity, 'Z-Up');
v0_charge7 = shell.Charge.MuzzleVelocity; % M4A2 CH7W (White bag, zone 7)

fprintf('Running Fire Control numerical shooting solver...\n');
sol = fcc.solveNumericalShooting(shell, launcher_pos, v0_charge7, target.getTargetBus(), env, 'Low', 0.005, 'FDR');

if ~sol.IsValid
    error('Target is out of range or solution failed to converge!');
end

fprintf('=== Solution Converged in %d Iterations ===\n', sol.Iterations);
fprintf('Quadrant Elevation (QE) : %.1f mils\n', sol.Elevation);
fprintf('Azimuth (Aim Bearing)   : %.1f mils (Wind corrected)\n', sol.Azimuth);
fprintf('Predicted Flight Time   : %.2f s\n', sol.TimeOfFlight);
fprintf('Convergence Miss Error  : %.3f m\n', sol.MissDistance);

% 5. Apply Solved Initial State to Munition. IForm/FL applied automatically
% inside buildLaunchState (see Munition.applyFormFactors).
shell.Dynamics.InitialState = shell.buildLaunchState(launcher_pos, v0_charge7, ...
    FireControlComputer.mil2rad(sol.Elevation), FireControlComputer.mil2rad(sol.Azimuth));
fprintf('Form Factors (at QE=%.1f mils, zone %s) : IForm=%.4f, FL=%.4f\n', ...
    sol.Elevation, shell.Charge.Name, shell.Aero.IForm, shell.Aero.FL);

% 6. Assemble Simulation & Execute
sim = Simulation(shell, target, env, [0 sol.TimeOfFlight * 1.5], 0.005);
sim.Frame = 'FDR';

fprintf('Starting trajectory integration (airbrake_ADT = %.2f s)...\n', airbrake_deploy_time);
tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.2f s simulated).\n', toc, telemetry.time(end));

% 7. Post-Flight Analysis
final_pos = telemetry.pos(end, :)';
final_tgt = telemetry.target_pos(end, :)';
actual_miss = norm(final_pos([1 3]) - final_tgt([1 3]));
[apogee, apogee_idx] = max(CoordinateFrame.altitudeColumn(telemetry.pos(:,2), telemetry.Frame));

fprintf('Termination Reason          : %s\n', sim.TerminationReason);
fprintf('Airbrake Deployed            : %s\n', mat2str(shell.Aero.Phase2Active));
fprintf('Apogee                      : %.1f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Actual Impact Miss Distance : %.2f meters\n', actual_miss);
fprintf('Forward Range               : %.0f m\n', final_pos(1));
fprintf('Deflection                  : %.0f m\n', final_pos(3));
fprintf('Total Range                 : %.0f m\n', hypot(final_pos(1), final_pos(3)));

plotTrajectoryProfile(telemetry, final_tgt, sprintf('M107 Artillery, airbrake @ %.1fs', airbrake_deploy_time));
