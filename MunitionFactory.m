%% File: MunitionFactory.m
classdef MunitionFactory
    methods (Static)
        
        %% 155mm M107 HE Artillery Projectile (Unguided Ballistic)
        function m = createM107Artillery()
            m = Munition('M107_155mm');
            
            % Mass properties (NATO standard ~43.2 kg, d = 0.155 m)
            m.MassProperties.Mass = 43.2;
            m.MassProperties.Inertia = diag([0.14, 1.62, 1.62]); % [Ix, Iy, Iz]
            
            % Subsystems
            m.Dynamics   = Dynamics3Dof();
            % S_ref = pi*(0.155/2)^2 = 0.01887 m^2, typical zero-lift drag CD0 ~ 0.22
            m.Aero       = Aero3DoF_Longitudinal(0.22, 2.5, 0.01887);
            m.Propulsion = []; % Pure ballistic, no motor
            m.Sensors    = {};
            m.Guidance   = [];
            m.Controller = [];
        end
        
        %% 2.75-inch (70mm) Hydra-70 Unguided Rocket (Mk 66 Motor)
        function m = createHydra70()
            m = Munition('Hydra_70_M151');
            
            % Launch mass ~10.5 kg (burnout mass ~6.2 kg)
            m.MassProperties.Mass = 10.5;
            m.MassProperties.Inertia = diag([0.008, 0.45, 0.45]);
            
            m.Dynamics   = Dynamics3Dof();
            % d = 0.070 m -> S_ref = 0.00385 m^2, CD0 ~ 0.38
            m.Aero       = Aero3DoF_Longitudinal(0.38, 3.2, 0.00385);
            % Mk 66 Mod 4: ~5900 N average thrust for ~1.1 seconds
            m.Propulsion = SolidMotor(5900.0, 1.1);
            m.Sensors    = {};
            m.Guidance   = [];
            m.Controller = [];
        end
        
        %% 25mm Bushmaster Target Practice Tracer (M793)
        function m = create25mmM793()
            m = Munition('25mm_M793');
            
            % Projectile mass ~0.185 kg, d = 0.025 m
            m.MassProperties.Mass = 0.185;
            m.MassProperties.Inertia = eye(3) * 1e-4;
            
            m.Dynamics   = Dynamics3Dof();
            % S_ref = pi*(0.025/2)^2 = 4.908e-4 m^2
            m.Aero       = Aero3DoF_Longitudinal(0.18, 1.5, 4.908e-4);
            m.Propulsion = [];
            m.Sensors    = {};
            m.Guidance   = [];
            m.Controller = [];
        end
        
        %% 35mm HE-T Anti-Aircraft/Anti-Drone Round (Unguided Direct-Fire)
        function m = create35mmHE()
            m = Munition('35mm_HET');

            % Illustrative mass properties for a 35mm AA gun round, d = 0.035 m
            m.MassProperties.Mass = 0.55;
            m.MassProperties.Inertia = diag([0.0004, 0.006, 0.006]);

            m.Dynamics = Dynamics3Dof();
            % S_ref = pi*(0.035/2)^2 = 9.62e-4 m^2, typical HE-T CD0 ~ 0.25
            m.Aero = Aero3DoF_Longitudinal(0.25, 2.0, 9.621e-4);
            m.Propulsion = []; % Pure ballistic, no motor
            m.Sensors    = {};
            m.Guidance   = [];
            m.Controller = [];
        end

        %% 76mm HE Projectile (Detailed Mach-Dependent Aero Table)
        function m = create76mmHE()
            m = Munition('76mm_HE');

            % Illustrative mass properties for a 76mm gun round, d = 0.076 m
            m.MassProperties.Mass = 6.0;
            m.MassProperties.Inertia = diag([0.006, 0.085, 0.085]);

            m.Dynamics = Dynamics3Dof();

            aero = AeroTabulated();
            aero.RefArea   = pi * (0.076/2)^2; % 0.004536 m^2
            aero.RefLength = 0.076;

            % Illustrative coefficient set - representative drag-bucket /
            % rising-then-falling lift-slope shape for a spin-stabilized
            % shell, NOT validated wind-tunnel or CFD data. Replace
            % Tables.Clean with real data when available. Deliberately
            % mixes all three representations BaseAero.evalCoeff supports:
            % CD0/CLA as Mach-Value tables, CDA2/CLA3 as polynomials in
            % Mach, CDA4 as a plain constant.
            coeffs.CD0 = struct('Type', 'Table', ...
                'Mach',  [0 0.7 0.9 0.95 1.0 1.05 1.1 1.2 1.5 2.0 2.5 3.0], ...
                'Value', [0.19 0.20 0.24 0.30 0.38 0.41 0.42 0.40 0.35 0.30 0.27 0.25]);

            coeffs.CDA2 = struct('Type', 'Polynomial', 'a0', 2.2, 'a1', 1.0, 'a2', -0.12);

            coeffs.CDA4 = 8.5; % plain scalar - constant across Mach

            coeffs.CLA = struct('Type', 'Table', ...
                'Mach',  [0 0.7 0.9 1.0 1.2 1.5 2.0 2.5 3.0], ...
                'Value', [2.0 2.1 2.4 2.8 2.6 2.3 2.0 1.8 1.6]);

            coeffs.CLA3 = struct('Type', 'Polynomial', 'a0', -8.0, 'a1', 1.5);

            aero.Tables.Clean   = coeffs;
            aero.ActiveTableKey = 'Clean';
            m.Aero = aero;

            m.Propulsion = []; % Pure ballistic, no motor
            m.Sensors    = {};
            m.Guidance   = [];
            m.Controller = [];
        end

        %% APKWS II (Laser-Guided 70mm Hydra variant)
        function m = createAPKWS(prf_code)
            if nargin < 1, prf_code = 1111; end
            
            % Start from Hydra base and add GNC kit
            m = MunitionFactory.createHydra70();
            m.Name = 'APKWS_Guided_Rocket';
            m.MassProperties.Mass = 15.0; % GNC mid-section adds weight
            
            % Attach LOAL Seeker and Guidance (LaserSeeker referenced here
            % previously does not exist anywhere in the codebase; SeekerLOAL
            % is the sensor OptimalAngleFOVGuidance/LiveEngagementPlotter
            % actually expect via sensor_bus.SeekerLOAL). prf_code is kept
            % as an input for future laser-designator integration but is
            % not consumed yet.
            m.Sensors    = { SeekerLOAL(deg2rad(20), 1.2) };
            guid = OptimalAngleFOVGuidance();
            guid.UseFOV = true;
            guid.T_SeekerOn = 1.2; % Seeker uncages after motor burnout
            m.Guidance   = guid;
            m.Controller = LongitudinalFinController();
        end
        
    end
end