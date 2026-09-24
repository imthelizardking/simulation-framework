%% Master Scenario: 155mm M107 Artillery (MPMM High-Fidelity Model)
% MunitionFactory.createM107Artillery now builds the high-fidelity MPMM
% pair (DynamicsMPMM + AeroMPMM) directly - AeroTabulatedSpin (the
% simpler model this preset used before MPMM was verified) has been
% removed - so this runner just calls the factory like any other preset,
% then adds the two things the factory can't know at construction time:
% launch latitude (a scenario choice) and the QE-dependent form factors
% (only knowable after the fire-control solve below).
clear; clc; clf;

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

% 4. Fire Control Computer (Solves Quadrant Elevation & Wind Crab Angle)
fcc = FireControlComputer(env.Gravity, 'Z-Up');
v0_charge7 = shell.Charge.MuzzleVelocity; % M4A2 CH7W (White bag, zone 7)

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

% 5. Apply Solved Initial State to Munition. IForm/FL (charge-zone- and
% QE-dependent, see MunitionData.formFactor) are applied automatically
% inside buildLaunchState now - see Munition.applyFormFactors - so
% there's no manual "look them up, then set Aero.IForm/FL" step here
% anymore; it's read back below purely to report what was applied.
shell.Dynamics.InitialState = shell.buildLaunchState(launcher_pos, v0_charge7, ...
    FireControlComputer.mil2rad(sol.Elevation), FireControlComputer.mil2rad(sol.Azimuth));
fprintf('Form Factors (at QE=%.1f mils, zone %s) : IForm=%.4f, FL=%.4f\n', ...
    sol.Elevation, shell.Charge.Name, shell.Aero.IForm, shell.Aero.FL);

% 6. Assemble Simulation & Execute
% NOTE: no Visualizer attached - LiveEngagementPlotter works fine against
% an unguided ballistic shell too (it degrades its seeker/FOV and command-
% breakdown panels gracefully when absent), but a post-flight static plot
% is still the better fit for a pure trajectory-generation scenario like
% this one - there's no live engagement decision-making to watch unfold.
sim = Simulation(shell, target, env, [0 sol.TimeOfFlight * 1.05], 0.005);
sim.Frame = 'FDR';

fprintf('Starting trajectory integration...\n');
tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.2f s simulated).\n', toc, telemetry.time(end));

% 7. Post-Flight Analysis
final_pos = telemetry.pos(end, :)';
% telemetry.target_pos, not target.Position - the latter is the raw
% Z-Up-sense value TargetStatic was constructed with, never rotated into
% the sim's own frame; telemetry.target_pos is Simulation's own per-step
% resolveTargetBus() output, already correctly rotated to match final_pos
% (see Simulation.m's class comment on Frame). Horizontal-plane (forward,
% right) miss, excluding the vertical component - right lives at index 3
% under 'FDR' (index 2 under 'NED'), not index 2 - see CoordinateFrame.m's
% class comment.
final_tgt = telemetry.target_pos(end, :)';
actual_miss = norm(final_pos([1 3]) - final_tgt([1 3]));
[apogee, apogee_idx] = max(CoordinateFrame.altitudeColumn(telemetry.pos(:,2), telemetry.Frame));

fprintf('Termination Reason          : %s\n', sim.TerminationReason);
fprintf('Apogee                      : %.1f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Actual Impact Miss Distance : %.2f meters\n', actual_miss);
fprintf('Forward Range               : %.0f m\n', final_pos(1));
fprintf('Deflection                  : %.0f m\n', final_pos(3));
fprintf('Total Range                 : %.0f m\n', hypot(final_pos(1), final_pos(3)));

plotTrajectoryProfile(telemetry, final_tgt, 'M107 Artillery Trajectory (MPMM)');
