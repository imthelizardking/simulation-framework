%% Runner: APKWS Compound 6-DoF Simulation
% Framework uyumlu, Simulation.m'in koşturduğu sarmalayıcı (wrapper) runner.

clear; clc; clf;

% 1. Setup Atmosphere
env = Atmosphere();

% 2. Setup Target
target = TargetConstantVelocity([4000; 500; 1500], [-200; 0; 0], 'NED');

% 3. Instantiate Vehicle (Doğrudan objelerle kuruyoruz)
missile = Munition('APKWS_Compound');
missile.Type = 'Guided';

geom.body_radius = 0.035;
geom.x_CG = 0.708;
geom.x_canard_AC = 0.5;
geom.x_tail_AC = 1.9;
geom.S_ref = pi*(0.035^2);
geom.S_canard = 0.005; geom.c_canard = 0.05;
geom.S_tail = 0.008;   geom.c_tail = 0.1;
geom.d = 0.07;

% 33 state'lik Compound Dynamics
missile.Dynamics = DynamicsCompound6DoF();

% Compound Aero Model
aero_tables = struct();
missile.Aero = AeroCompound6DoF(geom, aero_tables);
missile.Dynamics.AeroModel = missile.Aero; % İleride HM kullanımı için referans

missile.MassProperties.Mass = 15.0;
missile.MassProperties.Inertia = diag([0.015, 0.9, 0.9]);

missile.Sensors = { Seeker_3D() };
missile.Guidance = Guidance_ProportionalNav3D();
missile.Controller = AutopilotCompound(); % İçinde Mixer barındırır

% 4. Initial Launch State
launcher_pos = [0; 0; 0];
v0_rail = 50.0;
launch_elev = pi/4;
launch_az = 0;

% YENİ HALİ (Düzeltilmiş):
missile.Dynamics.InitialState = missile.Dynamics.buildLaunchState(...
    launcher_pos, v0_rail, launch_elev, launch_az, ...
    missile.MassProperties.Mass, 0, diag(missile.MassProperties.Inertia));

% 5. Assemble Simulation & Execute (Framework'ün kendi motoru)
sim = Simulation(missile, target, env, [0 30], 0.01);
sim.Frame = 'NED';
sim.Visualizer = Visualizer_LiveEngagement(5);

fprintf('Starting Compound 6-DoF engagement simulation...\n');
tic;
telemetry = sim.run(); % BÜTÜN İŞİ BURASI YAPAR
fprintf('Sim finished in %.2f seconds (%.2f s simulated).\n', toc, telemetry.time(end));

plotTrajectoryProfile(telemetry, telemetry.target_pos(end,:)', 'APKWS Compound 6-DoF');