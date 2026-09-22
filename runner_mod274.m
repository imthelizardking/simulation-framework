%% Scenario: MOD274 155mm Base-Bleed HE Artillery via MunitionFactory
clear; clc; close all;

% 1. Setup Environment
env = Environment(); % No wind - isolate the base-bleed effect

% 2. Setup Target: 25 km downrange - well beyond M107's ~21 km max range
%    at comparable charge energy, which is exactly the point of base bleed.
target = TargetStatic([25000; 0; 0], [], 'FDR'); % forward=25km, down=0, right=0

% 3. Instantiate Vehicle via MunitionFactory (DM92 6M = 943 m/s, the
%    munition's rated maximum; see MunitionFactory.createMOD274)
shell = MunitionFactory.createMOD274('6M');

% 4. Fire Control Computer (Solves Quadrant Elevation)
fcc = FireControlComputer(env.Gravity, 'Z-Up');
v0 = shell.Charge.MuzzleVelocity;
launcher_pos = [0; 0; 0];

fprintf('Firing MOD274 on %s charge %s (%.0f m/s)...\n', shell.Charge.Propellant, shell.Charge.Name, v0);
fprintf('Running Fire Control numerical shooting solver...\n');
sol = fcc.solveNumericalShooting(shell, launcher_pos, v0, target.getTargetBus(), env, 'Low');

if ~sol.IsValid
    error('Target is out of range or solution failed to converge!');
end

fprintf('=== Solution Converged in %d Iterations ===\n', sol.Iterations);
fprintf('Quadrant Elevation (QE) : %.1f mils\n', sol.Elevation);
fprintf('Predicted Flight Time   : %.2f s\n', sol.TimeOfFlight);
fprintf('Convergence Miss Error  : %.3f m\n', sol.MissDistance);

% 5. Apply Solved Initial State
shell.Dynamics.InitialState = shell.buildLaunchState(launcher_pos, v0, ...
    FireControlComputer.mil2rad(sol.Elevation), FireControlComputer.mil2rad(sol.Azimuth));

% 6. Assemble Simulation & Execute
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
bb = shell.Aero.BaseBleed;

fprintf('Termination Reason          : %s\n', sim.TerminationReason);
fprintf('Apogee                      : %.1f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Base Bleed Active Until     : %.1f s (%.0f%% CD0 reduction while burning)\n', ...
    bb.BurnDuration, bb.DragReductionFraction * 100);
fprintf('Actual Impact Miss Distance : %.2f meters\n', actual_miss);
fprintf('Forward Range               : %.0f m\n', final_pos(1));
fprintf('Deflection                  : %.0f m\n', final_pos(3));
fprintf('Total Range                 : %.0f m\n', hypot(final_pos(1), final_pos(3)));

plotTrajectoryProfile(telemetry, final_tgt, 'MOD274 Base-Bleed Trajectory');
