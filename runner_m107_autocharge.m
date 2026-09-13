%% Scenario: 155mm M107 Indirect Fire with Automatic Charge Selection
% Same idea as runner_m107.m, but instead of firing a fixed charge
% (Charge7) and solving only for elevation, this tries each available
% M119A2 charge zone (lowest to highest) via
% FireControlComputer.solveNumericalShootingAutoCharge and uses the first
% one that reaches the target - standard artillery doctrine of using the
% lowest sufficient charge rather than always maxing out. Target is closer
% than runner_m107.m's (which needs Charge7) specifically so a lower
% charge actually gets selected here - compare the two.
clear; clc; close all;

% 1. Setup Environment
env = Environment();

% 2. Setup Target - within Charge5's own max range (~10.4 km), so the
%    lowest zone should be the one selected.
target = TargetStatic([3000; 0; 0]);

% 3. Build one candidate munition per available M119A2 charge zone,
%    lowest to highest muzzle velocity (see ChargeFactory.getM119A2).
charge_names = {'Charge5', 'Charge6', 'Charge7'};
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

plotTrajectoryProfile(telemetry, target.Position, sprintf('M107 Auto-Charge (%s)', charge_names{chosen_idx}));
