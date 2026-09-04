%% Scenario: 76mm HE Projectile with Detailed Mach-Dependent Aero Table
clear; clc; close all;

% 1. Setup Environment (standard atmosphere, no wind)
env = Environment(1.225, 9.81, [0; 0; 0]);

% 2. Setup Target
target = StaticTarget([9000; 0; 0]); % 9 km downrange

% 3. Instantiate Vehicle via MunitionFactory (uses AeroTabulated - see
%    MunitionFactory.create76mmHE for the full CD0/CDA2/CDA4/CLA/CLA3
%    coefficient set)
shell = MunitionFactory.create76mmHE();

% 4. Fire Control Computer (Solves Quadrant Elevation)
fcc = FireControlComputer(env.Gravity, 'Z-Up');
v0 = 800.0; % m/s, muzzle velocity
launcher_pos = [0; 0; 0];

fprintf('Running Fire Control numerical shooting solver...\n');
sol = fcc.solveNumericalShooting(shell, launcher_pos, v0, target.getTargetBus(), env, 'Low');

if ~sol.IsValid
    error('Target is out of range or solution failed to converge!');
end

fprintf('=== Solution Converged in %d Iterations ===\n', sol.Iterations);
fprintf('Quadrant Elevation (QE) : %.2f deg\n', rad2deg(sol.Elevation));
fprintf('Predicted Flight Time   : %.2f s\n', sol.TimeOfFlight);
fprintf('Convergence Miss Error  : %.3f m\n', sol.MissDistance);

% 5. Apply Solved Initial State
v_x = v0 * cos(sol.Elevation);
v_z = v0 * sin(sol.Elevation);
shell.Dynamics.InitialState = [launcher_pos; v_x; 0; v_z; sol.Elevation; shell.MassProperties.Mass];

% 6. Assemble Simulation & Execute
sim = Simulation(shell, target, env, [0 sol.TimeOfFlight * 1.05], 0.005);

fprintf('Starting trajectory integration...\n');
tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.2f s simulated).\n', toc, telemetry.time(end));

% 7. Post-Flight Analysis
final_pos = telemetry.pos(end, :)';
actual_miss = norm(final_pos(1:2) - target.Position(1:2));
[apogee, apogee_idx] = max(telemetry.pos(:,3));
max_mach = max(vecnorm(telemetry.vel, 2, 2)) / env.SpeedOfSound;

fprintf('Termination Reason          : %s\n', sim.TerminationReason);
fprintf('Apogee                      : %.1f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Peak Mach Number            : %.2f\n', max_mach);
fprintf('Actual Impact Miss Distance : %.2f meters\n', actual_miss);

plotTrajectoryProfile(telemetry, target.Position, '76mm HE Trajectory (Tabulated Aero)');
