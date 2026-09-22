%% Scenario: 155mm M107 Indirect Fire with Automatic Charge Selection
% Same idea as runner_m107.m, but instead of firing a fixed charge
% (CH7W) and solving only for elevation, this tries each available
% M4A2 charge zone (lowest to highest) via
% FireControlComputer.solveNumericalShootingAutoCharge and uses the first
% one that reaches the target - standard artillery doctrine of using the
% lowest sufficient charge rather than always maxing out. Target is closer
% than runner_m107.m's (which needs CH7W) specifically so a lower
% charge actually gets selected here - compare the two.
clear; clc; close all;

% 1. Setup Environment
env = Environment();

% 2. Setup Target - within CH5W's own max range, so a lower-than-max
%    zone should be the one selected.
target = TargetStatic([3000; 0; 0], [], 'FDR'); % forward=3km, down=0, right=0

% 3. Build one candidate munition per available M4A2 charge zone,
%    lowest to highest muzzle velocity (see ChargeFactory.getM4A2).
charge_names = {'CH3W', 'CH4W', 'CH5W', 'CH6W', 'CH7W', 'CH7R'};
candidates = cell(1, numel(charge_names));
for i = 1:numel(charge_names)
    candidates{i} = MunitionFactory.createM107Artillery(charge_names{i});
end

% 4. Fire Control: try each charge, lowest first, use the first that
%    reaches the target.
fcc = FireControlComputer(env.Gravity, 'Z-Up');
launcher_pos = [0; 0; 0];
[sol, chosen_idx] = fcc.solveNumericalShootingAutoCharge(candidates, launcher_pos, target.getTargetBus(), env, 'Low');

if chosen_idx == 0
    error('No available charge reaches the target.');
end

shell = candidates{chosen_idx};
v0 = shell.Charge.MuzzleVelocity;

fprintf('=== Auto Charge Selection ===\n');
fprintf('Selected Charge         : %s (%.0f m/s)\n', charge_names{chosen_idx}, v0);
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

fprintf('Termination Reason          : %s\n', sim.TerminationReason);
fprintf('Apogee                      : %.1f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Actual Impact Miss Distance : %.2f meters\n', actual_miss);
fprintf('Forward Range               : %.0f m\n', final_pos(1));
fprintf('Deflection                  : %.0f m\n', final_pos(3));
fprintf('Total Range                 : %.0f m\n', hypot(final_pos(1), final_pos(3)));

plotTrajectoryProfile(telemetry, final_tgt, sprintf('M107 Auto-Charge (%s)', charge_names{chosen_idx}));
