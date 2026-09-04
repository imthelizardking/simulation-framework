%% File: run_minimissile_with_fcc.m
% Master Scenario: Mini-Missile with Fire Control Computer Lead & LOAL Guidance
clear; clc; close all;

% 1. Setup Target & Environment
% Correct object for moving targets
target = ManeuveringTarget([2000; 0; 800], [-150; 0; 0]);
target.LaserReflectivity = 0.3;
env    = Environment(1.225, 9.81, [0; 0; 0]);

% 2. Setup Vehicle
missile = Munition();
missile.Name = 'MiniMissile_Optimal';
missile.Dynamics   = Dynamics3DoF_PitchCoupled();
missile.Aero       = Aero3DoF_Longitudinal(0.35, 4.0, 0.01966);
missile.Propulsion = SolidMotor(1200.0, 2.0);

% Guidance and Sensor Configuration
seeker   = SeekerLOAL(deg2rad(20), 2.0);
guidance = OptimalAngleFOVGuidance();
guidance.UseFOV           = true;
guidance.UseTerminalAngle = false;
guidance.T_LOAL           = 0.5;
guidance.T_SeekerOn       = 2.0;

missile.Sensors    = { seeker };
missile.Guidance   = guidance;
missile.Controller = LongitudinalFinController();

% 3. Fire Control Solution Calculation
fcc = FireControlComputer(env.Gravity);
v0 = 111.8;       % Launch speed (m/s)
m0 = 3.0;         % Mass (kg)
launcher_pos = [0; 0; 0];

% Solve analytical firing angles for the moving target
sol = fcc.solveFiringAngles(launcher_pos, v0, target.getTargetBus(), 'DirectLead');

if ~sol.IsValid
    warning('DirectLead unreachable. Falling back to default high elevation launch.');
    launch_pitch = deg2rad(75.0);
    launch_yaw   = 0.0;
else
    launch_pitch = sol.Elevation;
    launch_yaw   = sol.Azimuth;
    fprintf('=== Firing Solution Calculated ===\n');
    fprintf('Solved Elevation : %.2f deg\n', rad2deg(launch_pitch));
    fprintf('Solved Azimuth   : %.2f deg\n', rad2deg(launch_yaw));
    fprintf('Estimated TOF    : %.2f sec\n', sol.TimeOfFlight);
end

% 4. Set Initial Launch State Vector (8x1)
% State: [x; y; z; vx; vy; vz; pitch; mass]
% Note: In this longitudinal 2D model (X-Z plane, Z-up), v_z = v0*sin(pitch)
v_x = v0 * cos(launch_pitch) * cos(launch_yaw);
v_y = v0 * cos(launch_pitch) * sin(launch_yaw);
v_z = v0 * sin(launch_pitch); 

missile.Dynamics.InitialState = [launcher_pos; v_x; v_y; v_z; launch_pitch; m0];
missile.MassProperties.Mass   = m0;

% 5. Assemble Simulation Orchestrator & Attach Visualizer
sim = Simulation(missile, target, env, [0 25], 0.001);
sim.Visualizer = LiveEngagementPlotter(10); % 10 ms decimation for smooth rendering

% 6. Execute Run
fprintf('Starting live simulation...\n');
tic;
telemetry = sim.run();
fprintf('Finished in %.2f seconds.\n', toc);