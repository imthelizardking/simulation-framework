%% File: MunitionFactory.m
classdef MunitionFactory
    methods (Static)
        
        %% 155mm M107 HE Artillery Projectile (Unguided Ballistic)
        function m = createM107Artillery(charge_name)
            if nargin < 1, charge_name = 'CH7W'; end
            % A data/createM107Artillery.mat
            % file overrides Name/Type/RefArea/TwistRate/etc below if
            % present (see FactoryData). Mass/Inertia/aero coefficients
            % instead come from data/<Name>.mat if present (see
            % MunitionData) - the new standardized per-munition data
            % file, which takes priority over this file's own hardcoded
            % defaults below.
            data = FactoryData.load('createM107Artillery');
            m = Munition(FactoryData.getOr(data, 'Name', 'M107_155mm'));
            m.Type = FactoryData.getOr(data, 'Type', 'HE');
            mdata = MunitionData.load(m.Name);
            aero_over = MunitionData.aeroCoeffs(mdata);

            % Mass properties (NATO standard ~43.2 kg, d = 0.155 m)
            m.MassProperties.Mass = MunitionData.mass(mdata, 43.2);
            m.MassProperties.Inertia = MunitionData.inertia(mdata, diag([0.14, 1.62, 1.62])); % [Ix, Iy, Iz]

            % Subsystems
            m.Dynamics   = DynamicsMPMM();
            % S_ref = pi*(0.155/2)^2 = 0.01887 m^2, typical zero-lift drag CD0 ~ 0.22.
            % Gun-fired and rifled - spin-stabilized, hence the high-
            % fidelity MPMM pair (DynamicsMPMM + AeroMPMM - full vector
            % yaw-of-repose, real gravity/Coriolis). Illustrative
            % magnitudes (McCoy-typical order), not validated wind-tunnel
            % data - same caveat as the rest of this file.
            aero = AeroMPMM();
            aero.RefArea   = FactoryData.getOr(data, 'RefArea', 0.01887);
            aero.RefLength = 0.155;
            aero.Ix        = m.MassProperties.Inertia(1,1); % must be kept in sync - see AeroMPMM.Ix
            aero.CD0   = FactoryData.getOr(aero_over, 'CD0',   0.22);
            aero.CDA2  = FactoryData.getOr(aero_over, 'CDA2',  0);
            aero.CDA4  = FactoryData.getOr(aero_over, 'CDA4',  0);
            aero.CLA   = FactoryData.getOr(aero_over, 'CLA',   2.5);
            aero.CLA3  = FactoryData.getOr(aero_over, 'CLA3',  0);
            aero.CLA5  = FactoryData.getOr(aero_over, 'CLA5',  0);
            aero.CSPIN = FactoryData.getOrAny(aero_over, {'CSPIN','CMASPIN'}, -0.03);
            aero.CMA   = FactoryData.getOr(aero_over, 'CMA',   2.0);
            aero.CMA3  = FactoryData.getOr(aero_over, 'CMA3',  0);
            aero.CMAG  = FactoryData.getOrAny(aero_over, {'CMAG','CMAGF'},  1.2);
            % IForm/FL left at AeroMPMM's neutral default (1.0) here -
            % they can be a polynomial in quadrant elevation (see
            % MunitionData.formFactor), which this factory method has no
            % way to know yet. The caller re-evaluates them once QE is
            % solved for, same two-pass pattern runner_m107.m uses.
            m.Aero = aero;
            m.TwistRate  = 20; % calibers/turn, illustrative (typical 155mm/39-cal tube figure)
            m.Charge     = ChargeFactory.getM4A2(charge_name);
            m.Propulsion = []; % Pure ballistic, no motor
            m.Sensors    = {};
            m.Guidance   = [];
            m.Controller = [];
        end
        
        %% 2.75-inch (70mm) Hydra-70 Unguided Rocket (Mk 66 Motor)
        function m = createHydra70()
            % A data/createHydra70.mat file
            % overrides Name/Type/RefArea/etc below if present (see
            % FactoryData); Mass/Inertia/CD0/CL_alpha instead come from
            % data/<Name>.mat if present (see MunitionData).
            data = FactoryData.load('createHydra70');
            m = Munition(FactoryData.getOr(data, 'Name', 'Hydra_70_M151'));
            m.Type = FactoryData.getOr(data, 'Type', 'Unguided');
            mdata = MunitionData.load(m.Name);
            aero_over = MunitionData.aeroCoeffs(mdata);

            % Launch mass ~10.5 kg (burnout mass ~6.2 kg)
            m.MassProperties.Mass = MunitionData.mass(mdata, 10.5);
            m.MassProperties.Inertia = MunitionData.inertia(mdata, diag([0.008, 0.45, 0.45]));

            m.Dynamics   = Dynamics3Dof();
            % d = 0.070 m -> S_ref = 0.00385 m^2, CD0 ~ 0.38
            m.Aero       = Aero3DoF_Longitudinal( ...
                FactoryData.getOr(aero_over, 'CD0', 0.38), ...
                FactoryData.getOr(aero_over, 'CL_alpha', 3.2), ...
                FactoryData.getOr(data, 'RefArea', 0.00385));
            % Mk 66 Mod 4: ~5900 N average thrust for ~1.1 seconds
            m.Propulsion = Propulsion_SolidMotor( ...
                FactoryData.getOr(data, 'Thrust', 5900.0), ...
                FactoryData.getOr(data, 'BurnTime', 1.1));
            % Rocket-motor propelled, not a gun-fired charge - Charge stays
            % empty (no propelling-charge concept applies here).
            m.Sensors    = {};
            m.Guidance   = [];
            m.Controller = [];
        end

        %% Fin-Stabilized Practice Dart (Basic Plain 3DoF Longitudinal Demo)
        function m = createFinStabilizedDart()
            % Deliberately the simplest munition in this factory: plain
            % Dynamics3Dof + Aero3DoF_Longitudinal, no spin, no charge
            % zones, no guidance - a clean baseline for the basic
            % pitch-plane lift/drag/pitch-follower model on its own,
            % unclouded by any of the spin-stabilized machinery the other
            % presets now use. Illustrative hand-thrown fin-stabilized
            % dart, d = 0.02 m.
            % A data/createFinStabilizedDart.mat
            % file overrides Name/Type/RefArea/etc below if present (see
            % FactoryData); Mass/Inertia/CD0/CL_alpha instead come from
            % data/<Name>.mat if present (see MunitionData).
            data = FactoryData.load('createFinStabilizedDart');
            m = Munition(FactoryData.getOr(data, 'Name', 'FinStabilized_Dart'));
            m.Type = FactoryData.getOr(data, 'Type', 'TP');
            mdata = MunitionData.load(m.Name);
            aero_over = MunitionData.aeroCoeffs(mdata);

            m.MassProperties.Mass = MunitionData.mass(mdata, 0.1);
            m.MassProperties.Inertia = MunitionData.inertia(mdata, eye(3) * 1e-5); % unused by Dynamics3Dof - no real attitude dynamics there

            m.Dynamics = Dynamics3Dof();
            % S_ref = pi*(0.02/2)^2 = 3.14e-4 m^2. CL_alpha is high relative
            % to the gun-fired shells above - tail fins give a strong,
            % genuinely-stabilizing lift-curve slope with no spin needed.
            m.Aero = Aero3DoF_Longitudinal( ...
                FactoryData.getOr(aero_over, 'CD0', 0.3), ...
                FactoryData.getOr(aero_over, 'CL_alpha', 5.0), ...
                FactoryData.getOr(data, 'RefArea', 3.14e-4));
            m.Charge = ChargeFactory.getFixed('Hand-Thrown', 30.0, 1.0);
            m.Propulsion = []; % Thrown, not fired or motor-propelled
            m.Sensors    = {};
            m.Guidance   = [];
            m.Controller = [];
        end

        %% 25mm Bushmaster Target Practice Tracer (M793)
        function m = create25mmM793()
            % A data/create25mmM793.mat file
            % overrides Name/Type/RefArea/etc below if present (see
            % FactoryData); Mass/Inertia/aero coefficients instead come
            % from data/<Name>.mat if present (see MunitionData).
            data = FactoryData.load('create25mmM793');
            m = Munition(FactoryData.getOr(data, 'Name', '25mm_M793'));
            m.Type = FactoryData.getOr(data, 'Type', 'TP');
            mdata = MunitionData.load(m.Name);
            aero_over = MunitionData.aeroCoeffs(mdata);

            % Projectile mass ~0.185 kg, d = 0.025 m
            m.MassProperties.Mass = MunitionData.mass(mdata, 0.185);
            m.MassProperties.Inertia = MunitionData.inertia(mdata, diag([1e-4, 1e-4, 1e-4]));

            m.Dynamics   = DynamicsMPMM();
            % S_ref = pi*(0.025/2)^2 = 4.908e-4 m^2. Rifled/spin-stabilized -
            % high-fidelity MPMM pair (DynamicsMPMM + AeroMPMM), same as
            % every other spin-stabilized preset in this file - see
            % createM107Artillery's comment.
            aero = AeroMPMM();
            aero.RefArea   = FactoryData.getOr(data, 'RefArea', 4.908e-4);
            aero.RefLength = FactoryData.getOr(data, 'RefLength', 0.025);
            aero.Ix        = m.MassProperties.Inertia(1,1);
            aero.CD0   = FactoryData.getOr(aero_over, 'CD0',   0.18);
            aero.CDA2  = FactoryData.getOr(aero_over, 'CDA2',  0);
            aero.CDA4  = FactoryData.getOr(aero_over, 'CDA4',  0);
            aero.CLA   = FactoryData.getOr(aero_over, 'CLA',   1.5);
            aero.CLA3  = FactoryData.getOr(aero_over, 'CLA3',  0);
            aero.CLA5  = FactoryData.getOr(aero_over, 'CLA5',  0);
            aero.CSPIN = FactoryData.getOrAny(aero_over, {'CSPIN','CMASPIN'}, -0.02);
            aero.CMA   = FactoryData.getOr(aero_over, 'CMA',   1.2);
            aero.CMA3  = FactoryData.getOr(aero_over, 'CMA3',  0);
            aero.CMAG  = FactoryData.getOrAny(aero_over, {'CMAG','CMAGF'},  0.6);
            m.Aero = aero;
            m.TwistRate  = FactoryData.getOr(data, 'TwistRate', 20); % calibers/turn, illustrative
            m.Charge     = ChargeFactory.getFixed('25mm-Fixed', 1100.0, 5.0);
            m.Propulsion = [];
            m.Sensors    = {};
            m.Guidance   = [];
            m.Controller = [];
        end

        %% 35mm HE-T Anti-Aircraft/Anti-Drone Round (Unguided Direct-Fire)
        function m = create35mmHE()
            % A data/create35mmHE.mat file
            % overrides Name/Type/RefArea/etc below if present (see
            % FactoryData); Mass/Inertia/aero coefficients instead come
            % from data/<Name>.mat if present (see MunitionData).
            data = FactoryData.load('create35mmHE');
            m = Munition(FactoryData.getOr(data, 'Name', '35mm_HET'));
            m.Type = FactoryData.getOr(data, 'Type', 'Airburst');
            % Backup fuze timer: bursts at this time regardless of any
            % programmed proximity/standoff burst time, so the round can't
            % fly on as unexploded ordnance if it never gets close enough
            % to the target to trigger (see Munition.SelfDestructTime).
            m.SelfDestructTime = FactoryData.getOr(data, 'SelfDestructTime', 8.0);
            mdata = MunitionData.load(m.Name);
            aero_over = MunitionData.aeroCoeffs(mdata);

            % Illustrative mass properties for a 35mm AA gun round, d = 0.035 m
            m.MassProperties.Mass = MunitionData.mass(mdata, 0.55);
            m.MassProperties.Inertia = MunitionData.inertia(mdata, diag([0.0004, 0.006, 0.006]));

            m.Dynamics = DynamicsMPMM();
            % S_ref = pi*(0.035/2)^2 = 9.62e-4 m^2, typical HE-T CD0 ~ 0.25.
            % Rifled/spin-stabilized - high-fidelity MPMM pair
            % (DynamicsMPMM + AeroMPMM), same as every other spin-
            % stabilized preset in this file - see
            % createM107Artillery's comment.
            aero = AeroMPMM();
            aero.RefArea   = FactoryData.getOr(data, 'RefArea', 9.621e-4);
            aero.RefLength = FactoryData.getOr(data, 'RefLength', 0.035);
            aero.Ix        = m.MassProperties.Inertia(1,1);
            aero.CD0   = FactoryData.getOr(aero_over, 'CD0',   0.25);
            aero.CDA2  = FactoryData.getOr(aero_over, 'CDA2',  0);
            aero.CDA4  = FactoryData.getOr(aero_over, 'CDA4',  0);
            aero.CLA   = FactoryData.getOr(aero_over, 'CLA',   2.0);
            aero.CLA3  = FactoryData.getOr(aero_over, 'CLA3',  0);
            aero.CLA5  = FactoryData.getOr(aero_over, 'CLA5',  0);
            aero.CSPIN = FactoryData.getOrAny(aero_over, {'CSPIN','CMASPIN'}, -0.025);
            aero.CMA   = FactoryData.getOr(aero_over, 'CMA',   1.6);
            aero.CMA3  = FactoryData.getOr(aero_over, 'CMA3',  0);
            aero.CMAG  = FactoryData.getOrAny(aero_over, {'CMAG','CMAGF'},  0.8);
            m.Aero = aero;
            m.TwistRate = FactoryData.getOr(data, 'TwistRate', 25); % calibers/turn, illustrative
            m.Charge = ChargeFactory.getFixed('35mm-Fixed', 1175.0, 4.0);
            m.Propulsion = []; % Pure ballistic, no motor
            m.Sensors    = {};
            m.Guidance   = [];
            m.Controller = [];
        end

        %% 76mm HE Projectile (Detailed Mach-Dependent Aero Table)
        function m = create76mmHE()
            % A data/create76mmHE.mat file
            % overrides Name/Type/RefArea/etc below if present (see
            % FactoryData); Mass/Inertia/aero coefficients instead come
            % from data/<Name>.mat if present (see MunitionData) -
            % including individual coefficients, so it can replace just
            % CD0 while CDA2/CLA/etc. keep their illustrative defaults.
            data = FactoryData.load('create76mmHE');
            m = Munition(FactoryData.getOr(data, 'Name', '76mm_HE'));
            m.Type = FactoryData.getOr(data, 'Type', 'Airburst');
            % Backup fuze timer: bursts at this time regardless of any
            % programmed proximity/standoff burst time, so the round can't
            % fly on as unexploded ordnance if it never gets close enough
            % to the target to trigger (see Munition.SelfDestructTime).
            m.SelfDestructTime = FactoryData.getOr(data, 'SelfDestructTime', 8.0);
            mdata = MunitionData.load(m.Name);
            aero_over = MunitionData.aeroCoeffs(mdata);

            % Illustrative mass properties for a 76mm gun round, d = 0.076 m
            m.MassProperties.Mass = MunitionData.mass(mdata, 6.0);
            m.MassProperties.Inertia = MunitionData.inertia(mdata, diag([0.006, 0.085, 0.085]));

            m.Dynamics = DynamicsMPMM();
            % High-fidelity MPMM pair (DynamicsMPMM + AeroMPMM), same as
            % every other spin-stabilized preset in this file - see
            % createM107Artillery's comment.
            aero = AeroMPMM();
            aero.RefArea   = FactoryData.getOr(data, 'RefArea', pi * (0.076/2)^2); % 0.004536 m^2
            aero.RefLength = FactoryData.getOr(data, 'RefLength', 0.076);
            aero.Ix        = m.MassProperties.Inertia(1,1);

            % Illustrative coefficient set - representative drag-bucket /
            % rising-then-falling lift-slope shape for a spin-stabilized
            % shell, NOT validated wind-tunnel or CFD data. Deliberately
            % mixes all three representations BaseAero.evalCoeff supports:
            % CD0/CLA as Mach-Value tables, CDA2/CLA3 as polynomials in
            % Mach, CDA4 as a plain constant.
            defaultCD0 = struct('Type', 'Table', ...
                'Mach',  [0 0.7 0.9 0.95 1.0 1.05 1.1 1.2 1.5 2.0 2.5 3.0], ...
                'Value', [0.19 0.20 0.24 0.30 0.38 0.41 0.42 0.40 0.35 0.30 0.27 0.25]);

            defaultCDA2 = struct('Type', 'Polynomial', 'a0', 2.2, 'a1', 1.0, 'a2', -0.12);

            defaultCLA = struct('Type', 'Table', ...
                'Mach',  [0 0.7 0.9 1.0 1.2 1.5 2.0 2.5 3.0], ...
                'Value', [2.0 2.1 2.4 2.8 2.6 2.3 2.0 1.8 1.6]);

            defaultCLA3 = struct('Type', 'Polynomial', 'a0', -8.0, 'a1', 1.5);

            aero.CD0  = FactoryData.getOr(aero_over, 'CD0', defaultCD0);
            aero.CDA2 = FactoryData.getOr(aero_over, 'CDA2', defaultCDA2);
            aero.CDA4 = FactoryData.getOr(aero_over, 'CDA4', 8.5); % plain scalar - constant across Mach
            aero.CLA  = FactoryData.getOr(aero_over, 'CLA', defaultCLA);
            aero.CLA3 = FactoryData.getOr(aero_over, 'CLA3', defaultCLA3);
            aero.CLA5 = FactoryData.getOr(aero_over, 'CLA5', 0);

            % Spin-stabilized extras - illustrative constants, same
            % caveat as the CD/CL buildup above.
            aero.CSPIN = FactoryData.getOrAny(aero_over, {'CSPIN','CMASPIN'}, -0.03);
            aero.CMA   = FactoryData.getOr(aero_over, 'CMA', 2.0);
            aero.CMA3  = FactoryData.getOr(aero_over, 'CMA3', 0);
            aero.CMAG  = FactoryData.getOrAny(aero_over, {'CMAG','CMAGF'}, 1.0);

            m.Aero = aero;
            m.TwistRate = FactoryData.getOr(data, 'TwistRate', 25); % calibers/turn, illustrative
            m.Charge = ChargeFactory.getFixed('76mm-Fixed', 800.0, 3.5);

            m.Propulsion = []; % Pure ballistic, no motor
            m.Sensors    = {};
            m.Guidance   = [];
            m.Controller = [];
        end

        %% MOD274 155mm Base-Bleed HE Projectile (Unguided, Extended Range)
        function m = createMOD274(charge_name)
            if nargin < 1, charge_name = '6M'; end

            % A data/createMOD274.mat file
            % overrides Name/Type/RefArea/etc below if present (see
            % FactoryData); Mass/Inertia/aero coefficients instead come
            % from data/<Name>.mat if present (see MunitionData) - same
            % standard coefficient vocabulary as every other artillery
            % preset (CD0/CDA2/CDA4/CLA/CLA3/CSPIN/CMA/CMA3/CMAG).
            data = FactoryData.load('createMOD274');
            m = Munition(FactoryData.getOr(data, 'Name', 'MOD274_155mm_BB'));
            m.Type = FactoryData.getOr(data, 'Type', 'HE');
            mdata = MunitionData.load(m.Name);
            aero_over = MunitionData.aeroCoeffs(mdata);

            % Illustrative mass properties for a 155mm base-bleed HE round
            % (runs a bit heavier than M107 due to the base-bleed unit)
            m.MassProperties.Mass = MunitionData.mass(mdata, 45.0);
            m.MassProperties.Inertia = MunitionData.inertia(mdata, diag([0.15, 1.7, 1.7]));

            m.Dynamics = DynamicsMPMM();
            % Same bore/S_ref as M107; slightly lower baseline CD0 is
            % typical of modern extended-range shell shaping even before
            % the base-bleed reduction is applied on top. Same rifled/
            % spin-stabilized treatment as M107 - high-fidelity MPMM pair
            % (DynamicsMPMM + AeroMPMM), illustrative CD0/CDA2/CDA4/CLA3/
            % CSPIN/CMA/CMA3/CMAG.
            aero = AeroMPMM();
            aero.RefArea   = FactoryData.getOr(data, 'RefArea', 0.01887);
            aero.RefLength = 0.155;
            aero.Ix        = m.MassProperties.Inertia(1,1);
            aero.CD0   = FactoryData.getOr(aero_over, 'CD0',   0.21);
            aero.CDA2  = FactoryData.getOr(aero_over, 'CDA2',  0);
            aero.CDA4  = FactoryData.getOr(aero_over, 'CDA4',  0);
            aero.CLA   = FactoryData.getOr(aero_over, 'CLA',   2.5);
            aero.CLA3  = FactoryData.getOr(aero_over, 'CLA3',  0);
            aero.CLA5  = FactoryData.getOr(aero_over, 'CLA5',  0);
            aero.CSPIN = FactoryData.getOrAny(aero_over, {'CSPIN','CMASPIN'}, -0.03);
            aero.CMA   = FactoryData.getOr(aero_over, 'CMA',   2.0);
            aero.CMA3  = FactoryData.getOr(aero_over, 'CMA3',  0);
            aero.CMAG  = FactoryData.getOrAny(aero_over, {'CMAG','CMAGF'},  1.2);
            % Very basic base-bleed model (see Propulsion_DYU): a fixed
            % fractional CD0 reduction for BurnDuration seconds, full drag
            % after burnout. Values are illustrative, not from a gas-
            % generator burn-rate model. (This is the CD0-multiplier
            % BaseBleed property, distinct from AeroMPMM's own optional
            % BaseBleedOn CG/inertia-shift refinement, which stays off -
            % see AeroMPMM.m.)
            aero.BaseBleed = Propulsion_DYU(30.0, 0.25);
            m.Aero = aero;
            m.TwistRate = FactoryData.getOr(data, 'TwistRate', 20); % calibers/turn, illustrative (same tube family as M107)
            m.Charge = ChargeFactory.getDM92(charge_name);

            m.Propulsion = []; % Pure ballistic, no gun-launch motor
            m.Sensors    = {};
            m.Guidance   = [];
            m.Controller = [];
        end

        %% APKWS II (Laser-Guided 70mm Hydra variant)
        function m = createAPKWS(prf_code)
            if nargin < 1, prf_code = 1111; end
            
            % Start from Hydra base and add GNC kit
            m = MunitionFactory.createHydra70();

            % A data/createAPKWS.mat file
            % overrides Name/Type/etc below if present (see FactoryData);
            % Mass instead comes from data/<Name>.mat if present (see
            % MunitionData) - note this is APKWS's OWN Name, a different
            % file from Hydra70's data/Hydra_70_M151.mat (if any), which
            % still applies underneath for the inherited Aero, since
            % createHydra70() is called above.
            data = FactoryData.load('createAPKWS');
            m.Name = FactoryData.getOr(data, 'Name', 'APKWS_Guided_Rocket');
            m.Type = FactoryData.getOr(data, 'Type', 'Guided');
            mdata = MunitionData.load(m.Name);
            m.MassProperties.Mass = MunitionData.mass(mdata, 15.0); % GNC mid-section adds weight

            % Attach LOAL Seeker and Guidance (LaserSeeker referenced here
            % previously does not exist anywhere in the codebase; SeekerLOAL
            % is the sensor OptimalAngleFOVGuidance/LiveEngagementPlotter
            % actually expect via sensor_bus.SeekerLOAL). prf_code is kept
            % as an input for future laser-designator integration but is
            % not consumed yet.
            seeker_fov_deg = FactoryData.getOr(data, 'SeekerFOV_deg', 20);
            t_seeker_on    = FactoryData.getOr(data, 'T_SeekerOn', 1.2); % Seeker uncages after motor burnout
            m.Sensors    = { SeekerLOAL(deg2rad(seeker_fov_deg), t_seeker_on) };
            guid = OptimalAngleFOVGuidance();
            guid.UseFOV = true;
            guid.T_SeekerOn = t_seeker_on;
            m.Guidance   = guid;
            m.Controller = LongitudinalFinController();
        end

        %% Guided Rocket with Missile DATCOM Aero Tables (70mm-class)
        function m = createGuidedRocketDatcom(datcom_name)
            if nargin < 1, datcom_name = 'ex1_in'; end
            % A data/createGuidedRocketDatcom.<ext> file overrides the
            % mass/motor/seeker parameters below if present (see
            % FactoryData); the aero table itself is a SEPARATE Missile
            % DATCOM printed-output file (data/<datcom_name>.txt, see
            % DatcomReader) rather than a FactoryData override, since it's
            % a 2D Mach/Alpha grid, not the scalar/Mach-table coefficient
            % shape FactoryData.getOr(...) is built for. Defaults to
            % data/ex1_in.txt - a real Digital DATCOM run (the manual's
            % "Example Problem 1"; data/EX1_inout.txt is its input deck)
            % - rather than fabricated data, so the pipeline is exercised
            % against genuine DATCOM output. Swap in your own rocket's
            % DATCOM run once you have one; its reference dimensions will
            % differ from the defaults below.
            data = FactoryData.load('createGuidedRocketDatcom');
            m = Munition(FactoryData.getOr(data, 'Name', 'GuidedRocket_DATCOM'));
            m.Type = FactoryData.getOr(data, 'Type', 'Guided');
            mdata = MunitionData.load(m.Name);

            % Illustrative mass properties (not from the DATCOM run, which
            % only reports aero) - adjust to your actual rocket's real
            % mass/inertia, or supply data/<Name>.mat (see MunitionData).
            m.MassProperties.Mass = MunitionData.mass(mdata, 15.0);
            m.MassProperties.Inertia = MunitionData.inertia(mdata, diag([0.008, 0.45, 0.45]));

            m.Dynamics = Dynamics3Dof();
            % RefArea/RefLength default to Example Problem 1's own SREF/
            % CBARR (8.85 ft^2, 2.48 ft - see data/EX1_inout.txt's $OPTINS
            % card), converted to SI: the CD/CL table is only physically
            % meaningful with the reference dimensions DATCOM normalized
            % it against.
            m.Aero = DatcomReader.load(datcom_name, ...
                FactoryData.getOr(data, 'RefArea', 0.822192), ...   % 8.85 ft^2
                FactoryData.getOr(data, 'RefLength', 0.755904));    % 2.48 ft
            m.Propulsion = Propulsion_SolidMotor( ...
                FactoryData.getOr(data, 'Thrust', 5900.0), ...
                FactoryData.getOr(data, 'BurnTime', 1.1));

            seeker_fov_deg = FactoryData.getOr(data, 'SeekerFOV_deg', 20);
            t_seeker_on    = FactoryData.getOr(data, 'T_SeekerOn', 1.2); % Seeker uncages after motor burnout
            m.Sensors    = { SeekerLOAL(deg2rad(seeker_fov_deg), t_seeker_on) };
            guid = OptimalAngleFOVGuidance();
            guid.UseFOV = true;
            guid.T_SeekerOn = t_seeker_on;
            m.Guidance   = guid;
            m.Controller = LongitudinalFinController();
        end

    end
end