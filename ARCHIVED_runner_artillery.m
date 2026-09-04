%% File: run_artillery_scenario.m
clear; clc; close all;

% -------------------------------------------------------------
% 1. Setup Environment
% -------------------------------------------------------------
env = Environment(1.225, 9.81, [0; 0; 0]); % Can include crosswind here

% -------------------------------------------------------------
% 2. Setup Vehicle (Munition + Aero + Motor)
% -------------------------------------------------------------
missile = Munition('Unguided_Artillery');
missile.Dynamics   = Dynamics3Dof();
missile.Aero       = Aero3DoF_Longitudinal(0.35, 4.0, 0.01966);
missile.Propulsion = SolidMotor(1200.0, 2.0); % Or [] for unpowered artillery shell
missile.MassProperties.Mass = 3.0;

% =============================================================
% >>> PLACE THE FIRE CONTROL COMPUTER BLOCK HERE <<<
% =============================================================

% Target at 7500 meters downrange
target = StaticTarget([2500; 200; 0]); 

% Initialize Fire Control Computer
fcc = FireControlComputer(env.Gravity, 'Z-Up');

v0 = 250.0; % Launch / Muzzle velocity (m/s)
launcher_pos = [0; 0; 0];

% Run Shooting Inversion Solver using vehicle's true Aero & Motor
sol = fcc.solveNumericalShooting(missile, launcher_pos, v0, target.getTargetBus(), env, 'Low');

if sol.IsValid
    fprintf('=== Precision Shooting Solution ===\n');
    fprintf('Converged in %d iterations\n', sol.Iterations);
    fprintf('Final Miss Distance : %.3f meters\n', sol.MissDistance);
    fprintf('Quadrant Elevation  : %.2f deg\n', rad2deg(sol.Elevation));
    fprintf('Azimuth             : %.2f deg\n', rad2deg(sol.Azimuth));
    fprintf('Time of Flight      : %.2f sec\n', sol.TimeOfFlight);
    
    launch_pitch = sol.Elevation;
    launch_yaw   = sol.Azimuth;
else
    error('Target is outside the physical engagement envelope for given V0!');
end

% -------------------------------------------------------------
% 3. Apply Solved Angles to Initial Conditions
% -------------------------------------------------------------
v_x = v0 * cos(launch_pitch) * cos(launch_yaw);
v_y = v0 * cos(launch_pitch) * sin(launch_yaw);
v_z = v0 * sin(launch_pitch);

missile.Dynamics.InitialState = [launcher_pos; v_x; v_y; v_z; launch_pitch; missile.MassProperties.Mass];

% -------------------------------------------------------------
% 4. Assemble Simulation Orchestrator & Run
% -------------------------------------------------------------
sim = Simulation(missile, target, env, [0 sol.TimeOfFlight * 1.1], 0.001);
sim.Visualizer = LiveEngagementPlotter(10);

fprintf('Starting high-precision simulation...\n');
telemetry = sim.run();