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
            m.Dynamics   = Dynamics3DoF_PitchCoupled();
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
            
            m.Dynamics   = Dynamics3DoF_PitchCoupled();
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
            
            m.Dynamics   = Dynamics3DoF_PitchCoupled();
            % S_ref = pi*(0.025/2)^2 = 4.908e-4 m^2
            m.Aero       = Aero3DoF_Longitudinal(0.18, 1.5, 4.908e-4);
            m.Propulsion = [];
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