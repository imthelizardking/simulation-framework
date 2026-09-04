%% Master Scenario: Mini-Missile with LOAL Seeker & Optimal Guidance
clear; clc; close all;

% 1. Setup Target & Environment
target = struct('pos', [2000; 0; 800], 'vel', [-150; 0; 0]);
env    = Environment(1.225, 9.81, [0; 0; 0]);

% 2. Setup Vehicle
missile = Munition();
missile.Name = 'MiniMissile_Optimal';
missile.Dynamics   = Dynamics3Dof();
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

% 3. Set Initial Launch State (Speed: 111.8 m/s, Launch Pitch: 75 deg, Mass: 3 kg)
v0 = 111.8;
launch_pitch = deg2rad(75.0);
missile.Dynamics.InitialState = [0; 0; 0; v0*cos(launch_pitch); 0; v0*sin(launch_pitch); launch_pitch; 3.0];
missile.MassProperties.Mass   = 3.0;

% 4. Assemble Simulation Orchestrator & Attach Visualizer
sim = Simulation(missile, target, env, [0 25], 0.001);
sim.Visualizer = LiveEngagementPlotter(1); % Update UI every 10 ms (100 Hz refresh)

% 5. Execute Run
fprintf('Starting live simulation...\n');
tic;
telemetry = sim.run();
fprintf('Finished in %.2f seconds.\n', toc);