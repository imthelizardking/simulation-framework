%% File: MunitionFactory.m
classdef MunitionFactory
    methods (Static)
        
        %% 155mm M107 HE Artillery Projectile (Unguided Ballistic)
        function m = createM107Artillery(charge_name)
            if nargin < 1, charge_name = 'Charge7'; end
            % A data/createM107Artillery.<json|xml|fci|xlsx|mat|txt|sldd>
            % file overrides the parameters below if present (see
            % FactoryData); otherwise these hardcoded values are used as-is.
            data = FactoryData.load('createM107Artillery');
            m = Munition(FactoryData.getOr(data, 'Name', 'M107_155mm'));

            % Mass properties (NATO standard ~43.2 kg, d = 0.155 m)
            m.MassProperties.Mass = FactoryData.getOr(data, 'Mass', 43.2);
            m.MassProperties.Inertia = diag(FactoryData.getOr(data, 'Inertia', [0.14, 1.62, 1.62])); % [Ix, Iy, Iz]

            % Subsystems
            m.Dynamics   = Dynamics3DofSpin();
            % S_ref = pi*(0.155/2)^2 = 0.01887 m^2, typical zero-lift drag CD0 ~ 0.22.
            % Gun-fired and rifled - spin-stabilized, hence AeroTabulatedSpin
            % rather than the plain Aero3DoF_Longitudinal: CDA2/CDA4/CLA3 are
            % zeroed out to reproduce the old CD0/CL_alpha-only buildup
            % exactly, with Cl_p/CMagnus_pa added on top for spin damping and
            % Magnus drift. Illustrative magnitudes (McCoy-typical order), not
            % validated wind-tunnel data - same caveat as the rest of this file.
            aero = AeroTabulatedSpin();
            aero.RefArea   = FactoryData.getOr(data, 'RefArea', 0.01887);
            aero.RefLength = 0.155;
            coeffs.CD0   = FactoryData.getOr(data, 'CD0', 0.22);
            coeffs.CDA2  = FactoryData.getOr(data, 'CDA2', 0);
            coeffs.CDA4  = FactoryData.getOr(data, 'CDA4', 0);
            coeffs.CLA   = FactoryData.getOr(data, 'CLA', 2.5);
            coeffs.CLA3  = FactoryData.getOr(data, 'CLA3', 0);
            coeffs.CSPIN = FactoryData.getOr(data, 'CSPIN', -0.03);
            coeffs.CMA   = FactoryData.getOr(data, 'CMA', 2.0);
            coeffs.CMA3  = FactoryData.getOr(data, 'CMA3', 0);
            coeffs.CMAG  = FactoryData.getOr(data, 'CMAG', 1.2);
            aero.Tables.Clean   = coeffs;
            aero.ActiveTableKey = 'Clean';
            m.Aero = aero;
            m.TwistRate  = 20; % calibers/turn, illustrative (typical 155mm/39-cal tube figure)
            m.Charge     = ChargeFactory.getM119A2(charge_name);
            m.Propulsion = []; % Pure ballistic, no motor
            m.Sensors    = {};
            m.Guidance   = [];
            m.Controller = [];
        end
        
        %% 2.75-inch (70mm) Hydra-70 Unguided Rocket (Mk 66 Motor)
        function m = createHydra70()
            % A data/createHydra70.<json|xml|fci|xlsx|mat|txt|sldd> file
            % overrides the parameters below if present (see FactoryData);
            % otherwise these hardcoded values are used as-is.
            data = FactoryData.load('createHydra70');
            m = Munition(FactoryData.getOr(data, 'Name', 'Hydra_70_M151'));

            % Launch mass ~10.5 kg (burnout mass ~6.2 kg)
            m.MassProperties.Mass = FactoryData.getOr(data, 'Mass', 10.5);
            m.MassProperties.Inertia = diag(FactoryData.getOr(data, 'Inertia', [0.008, 0.45, 0.45]));

            m.Dynamics   = Dynamics3Dof();
            % d = 0.070 m -> S_ref = 0.00385 m^2, CD0 ~ 0.38
            m.Aero       = Aero3DoF_Longitudinal( ...
                FactoryData.getOr(data, 'CD0', 0.38), ...
                FactoryData.getOr(data, 'CL_alpha', 3.2), ...
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
            % A data/createFinStabilizedDart.<json|xml|fci|xlsx|mat|txt|sldd>
            % file overrides the parameters below if present (see
            % FactoryData); otherwise these hardcoded values are used as-is.
            data = FactoryData.load('createFinStabilizedDart');
            m = Munition(FactoryData.getOr(data, 'Name', 'FinStabilized_Dart'));

            m.MassProperties.Mass = FactoryData.getOr(data, 'Mass', 0.1);
            m.MassProperties.Inertia = eye(3) * 1e-5; % unused by Dynamics3Dof - no real attitude dynamics there

            m.Dynamics = Dynamics3Dof();
            % S_ref = pi*(0.02/2)^2 = 3.14e-4 m^2. CL_alpha is high relative
            % to the gun-fired shells above - tail fins give a strong,
            % genuinely-stabilizing lift-curve slope with no spin needed.
            m.Aero = Aero3DoF_Longitudinal( ...
                FactoryData.getOr(data, 'CD0', 0.3), ...
                FactoryData.getOr(data, 'CL_alpha', 5.0), ...
                FactoryData.getOr(data, 'RefArea', 3.14e-4));
            m.Charge = ChargeFactory.getFixed('Hand-Thrown', 30.0, 1.0);
            m.Propulsion = []; % Thrown, not fired or motor-propelled
            m.Sensors    = {};
            m.Guidance   = [];
            m.Controller = [];
        end

        %% 25mm Bushmaster Target Practice Tracer (M793)
        function m = create25mmM793()
            % A data/create25mmM793.<json|xml|fci|xlsx|mat|txt|sldd> file
            % overrides the parameters below if present (see FactoryData);
            % otherwise these hardcoded values are used as-is.
            data = FactoryData.load('create25mmM793');
            m = Munition(FactoryData.getOr(data, 'Name', '25mm_M793'));

            % Projectile mass ~0.185 kg, d = 0.025 m
            m.MassProperties.Mass = FactoryData.getOr(data, 'Mass', 0.185);
            m.MassProperties.Inertia = diag(FactoryData.getOr(data, 'Inertia', [1e-4, 1e-4, 1e-4]));

            m.Dynamics   = Dynamics3DofSpin();
            % S_ref = pi*(0.025/2)^2 = 4.908e-4 m^2. Rifled/spin-stabilized -
            % AeroTabulatedSpin with CDA2/CDA4/CLA3 zeroed reproduces the old
            % CD0/CL_alpha-only buildup, plus illustrative Cl_p/CMagnus_pa.
            aero = AeroTabulatedSpin();
            aero.RefArea   = FactoryData.getOr(data, 'RefArea', 4.908e-4);
            aero.RefLength = FactoryData.getOr(data, 'RefLength', 0.025);
            coeffs.CD0   = FactoryData.getOr(data, 'CD0', 0.18);
            coeffs.CDA2  = FactoryData.getOr(data, 'CDA2', 0);
            coeffs.CDA4  = FactoryData.getOr(data, 'CDA4', 0);
            coeffs.CLA   = FactoryData.getOr(data, 'CLA', 1.5);
            coeffs.CLA3  = FactoryData.getOr(data, 'CLA3', 0);
            coeffs.CSPIN = FactoryData.getOr(data, 'CSPIN', -0.02);
            coeffs.CMA   = FactoryData.getOr(data, 'CMA', 1.2);
            coeffs.CMA3  = FactoryData.getOr(data, 'CMA3', 0);
            coeffs.CMAG  = FactoryData.getOr(data, 'CMAG', 0.6);
            aero.Tables.Clean   = coeffs;
            aero.ActiveTableKey = 'Clean';
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
            % A data/create35mmHE.<json|xml|fci|xlsx|mat|txt|sldd> file
            % overrides the parameters below if present (see FactoryData);
            % otherwise these hardcoded values are used as-is.
            data = FactoryData.load('create35mmHE');
            m = Munition(FactoryData.getOr(data, 'Name', '35mm_HET'));

            % Illustrative mass properties for a 35mm AA gun round, d = 0.035 m
            m.MassProperties.Mass = FactoryData.getOr(data, 'Mass', 0.55);
            m.MassProperties.Inertia = diag(FactoryData.getOr(data, 'Inertia', [0.0004, 0.006, 0.006]));

            m.Dynamics = Dynamics3DofSpin();
            % S_ref = pi*(0.035/2)^2 = 9.62e-4 m^2, typical HE-T CD0 ~ 0.25.
            % Rifled/spin-stabilized - AeroTabulatedSpin with CDA2/CDA4/CLA3
            % zeroed reproduces the old CD0/CL_alpha-only buildup, plus
            % illustrative Cl_p/CMagnus_pa.
            aero = AeroTabulatedSpin();
            aero.RefArea   = FactoryData.getOr(data, 'RefArea', 9.621e-4);
            aero.RefLength = FactoryData.getOr(data, 'RefLength', 0.035);
            coeffs.CD0   = FactoryData.getOr(data, 'CD0', 0.25);
            coeffs.CDA2  = FactoryData.getOr(data, 'CDA2', 0);
            coeffs.CDA4  = FactoryData.getOr(data, 'CDA4', 0);
            coeffs.CLA   = FactoryData.getOr(data, 'CLA', 2.0);
            coeffs.CLA3  = FactoryData.getOr(data, 'CLA3', 0);
            coeffs.CSPIN = FactoryData.getOr(data, 'CSPIN', -0.025);
            coeffs.CMA   = FactoryData.getOr(data, 'CMA', 1.6);
            coeffs.CMA3  = FactoryData.getOr(data, 'CMA3', 0);
            coeffs.CMAG  = FactoryData.getOr(data, 'CMAG', 0.8);
            aero.Tables.Clean   = coeffs;
            aero.ActiveTableKey = 'Clean';
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
            % A data/create76mmHE.<json|xml|fci|xlsx|mat|txt|sldd> file
            % overrides any of the parameters below if present (see
            % FactoryData) - including individual coefficients, so a file
            % can replace just CD0 with real data while CDA2/CLA/etc. keep
            % their illustrative defaults.
            data = FactoryData.load('create76mmHE');
            m = Munition(FactoryData.getOr(data, 'Name', '76mm_HE'));

            % Illustrative mass properties for a 76mm gun round, d = 0.076 m
            m.MassProperties.Mass = FactoryData.getOr(data, 'Mass', 6.0);
            m.MassProperties.Inertia = diag(FactoryData.getOr(data, 'Inertia', [0.006, 0.085, 0.085]));

            m.Dynamics = Dynamics3DofSpin();

            aero = AeroTabulatedSpin();
            aero.RefArea   = FactoryData.getOr(data, 'RefArea', pi * (0.076/2)^2); % 0.004536 m^2
            aero.RefLength = FactoryData.getOr(data, 'RefLength', 0.076);

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

            coeffs.CD0  = FactoryData.getOr(data, 'CD0', defaultCD0);
            coeffs.CDA2 = FactoryData.getOr(data, 'CDA2', defaultCDA2);
            coeffs.CDA4 = FactoryData.getOr(data, 'CDA4', 8.5); % plain scalar - constant across Mach
            coeffs.CLA  = FactoryData.getOr(data, 'CLA', defaultCLA);
            coeffs.CLA3 = FactoryData.getOr(data, 'CLA3', defaultCLA3);

            % Spin-stabilized extras (AeroTabulatedSpin) - illustrative
            % constants, same caveat as the CD/CL buildup above.
            coeffs.CSPIN = FactoryData.getOr(data, 'CSPIN', -0.03);
            coeffs.CMA   = FactoryData.getOr(data, 'CMA', 2.0);
            coeffs.CMA3  = FactoryData.getOr(data, 'CMA3', 0);
            coeffs.CMAG  = FactoryData.getOr(data, 'CMAG', 1.0);

            aero.Tables.Clean   = coeffs;
            aero.ActiveTableKey = 'Clean';
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

            % A data/createMOD274.<json|xml|fci|xlsx|mat|txt|sldd> file
            % overrides the parameters below if present (see FactoryData) -
            % same standard coefficient vocabulary as every other artillery
            % preset (CD0/CDA2/CDA4/CLA/CLA3/CSPIN/CMA/CMA3/CMAG).
            data = FactoryData.load('createMOD274');
            m = Munition(FactoryData.getOr(data, 'Name', 'MOD274_155mm_BB'));

            % Illustrative mass properties for a 155mm base-bleed HE round
            % (runs a bit heavier than M107 due to the base-bleed unit)
            m.MassProperties.Mass = FactoryData.getOr(data, 'Mass', 45.0);
            m.MassProperties.Inertia = diag(FactoryData.getOr(data, 'Inertia', [0.15, 1.7, 1.7]));

            m.Dynamics = Dynamics3DofSpin();
            % Same bore/S_ref as M107; slightly lower baseline CD0 is
            % typical of modern extended-range shell shaping even before
            % the base-bleed reduction is applied on top. Same rifled/
            % spin-stabilized treatment as M107 - AeroTabulatedSpin with
            % CDA2/CDA4/CLA3 zeroed reproduces the old CD0/CL_alpha-only
            % buildup, plus illustrative CSPIN/CMA/CMA3/CMAG.
            aero = AeroTabulatedSpin();
            aero.RefArea   = FactoryData.getOr(data, 'RefArea', 0.01887);
            aero.RefLength = 0.155;
            coeffs.CD0   = FactoryData.getOr(data, 'CD0', 0.21);
            coeffs.CDA2  = FactoryData.getOr(data, 'CDA2', 0);
            coeffs.CDA4  = FactoryData.getOr(data, 'CDA4', 0);
            coeffs.CLA   = FactoryData.getOr(data, 'CLA', 2.5);
            coeffs.CLA3  = FactoryData.getOr(data, 'CLA3', 0);
            coeffs.CSPIN = FactoryData.getOr(data, 'CSPIN', -0.03);
            coeffs.CMA   = FactoryData.getOr(data, 'CMA', 2.0);
            coeffs.CMA3  = FactoryData.getOr(data, 'CMA3', 0);
            coeffs.CMAG  = FactoryData.getOr(data, 'CMAG', 1.2);
            aero.Tables.Clean   = coeffs;
            aero.ActiveTableKey = 'Clean';
            % Very basic base-bleed model (see Propulsion_DYU): a fixed
            % fractional CD0 reduction for BurnDuration seconds, full drag
            % after burnout. Values are illustrative, not from a gas-
            % generator burn-rate model.
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

            % A data/createAPKWS.<json|xml|fci|xlsx|mat|txt|sldd> file
            % overrides the parameters below if present (see FactoryData);
            % otherwise these hardcoded values are used as-is. Hydra70's
            % own data/createHydra70.<ext> file (if any) still applies
            % underneath, since createHydra70() is called above.
            data = FactoryData.load('createAPKWS');
            m.Name = FactoryData.getOr(data, 'Name', 'APKWS_Guided_Rocket');
            m.MassProperties.Mass = FactoryData.getOr(data, 'Mass', 15.0); % GNC mid-section adds weight

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

    end
end