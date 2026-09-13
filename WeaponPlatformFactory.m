classdef WeaponPlatformFactory
    % Illustrative platform presets - representative traverse limits and
    % relative dispersion/rate-of-fire differences between platform
    % classes, not validated manufacturer data. MaxMuzzleVelocity is the
    % weapon's own rated ceiling (independent of which munition/charge is
    % loaded); pair with a MunitionFactory preset of matching caliber via
    % FireMission, whose Charge.MuzzleVelocity supplies the actual
    % per-shot velocity and gets checked against this ceiling.
    methods (Static)

        %% Towed 155mm Howitzer (e.g. M198/M777-class)
        function w = createTowedHowitzer155mm()
            w = WeaponPlatform();
            w.Name = 'Towed_155mm_Howitzer';
            w.MaxMuzzleVelocity = 900.0; % headroom above M119A2's top zone (Charge7, 580 m/s)
            w.ElevationMin = FireControlComputer.rad2mil(deg2rad(0.0));
            w.ElevationMax = FireControlComputer.rad2mil(deg2rad(72.0));
            w.AzimuthMin   = FireControlComputer.rad2mil(deg2rad(-22.5)); % on-carriage traverse without repositioning the trails
            w.AzimuthMax   = FireControlComputer.rad2mil(deg2rad(22.5));
            w.ElevationJitterStdDev = FireControlComputer.rad2mil(deg2rad(0.08)); % less consistent settling than an SPG
            w.AzimuthJitterStdDev   = FireControlComputer.rad2mil(deg2rad(0.08));
            w.RateOfFire = 3.0; % rounds/min, sustained
        end

        %% T-155 Firtina SPG (K9 Thunder-based turret)
        function w = createFirtinaSPG()
            w = WeaponPlatform();
            w.Name = 'Firtina_155mm_SPG';
            w.MaxMuzzleVelocity = 960.0; % headroom above DM92 6M (943 m/s)
            w.ElevationMin = FireControlComputer.rad2mil(deg2rad(-3.0));
            w.ElevationMax = FireControlComputer.rad2mil(deg2rad(65.0));
            w.AzimuthMin = -3200; % full 360 deg turret traverse
            w.AzimuthMax = 3200;
            w.ElevationJitterStdDev = FireControlComputer.rad2mil(deg2rad(0.04)); % tighter than towed - autoloader + digital fire control
            w.AzimuthJitterStdDev   = FireControlComputer.rad2mil(deg2rad(0.04));
            w.RateOfFire = 6.0; % rounds/min, sustained
        end

        %% 35mm Twin AA Mount (Oerlikon-class)
        function w = create35mmAAMount()
            w = WeaponPlatform();
            w.Name = 'AA_35mm_Twin_Mount';
            w.MaxMuzzleVelocity = 1200.0; % headroom above the 35mm fixed charge (1175 m/s)
            w.ElevationMin = FireControlComputer.rad2mil(deg2rad(-5.0));
            w.ElevationMax = FireControlComputer.rad2mil(deg2rad(92.0)); % near-vertical/overhead engagement
            w.AzimuthMin = -3200; % full traverse, radar-directed
            w.AzimuthMax = 3200;
            w.ElevationJitterStdDev = FireControlComputer.rad2mil(deg2rad(0.05));
            w.AzimuthJitterStdDev   = FireControlComputer.rad2mil(deg2rad(0.05));
            w.RateOfFire = 550.0; % rounds/min, cyclic
        end

        %% 76mm Naval/Vehicle Mount (OTO Melara-class)
        function w = create76mmNavalMount()
            w = WeaponPlatform();
            w.Name = 'Naval_76mm_Mount';
            w.MaxMuzzleVelocity = 850.0; % headroom above the 76mm fixed charge (800 m/s)
            w.ElevationMin = FireControlComputer.rad2mil(deg2rad(-15.0));
            w.ElevationMax = FireControlComputer.rad2mil(deg2rad(85.0));
            w.AzimuthMin = -3200; % full traverse
            w.AzimuthMax = 3200;
            w.ElevationJitterStdDev = FireControlComputer.rad2mil(deg2rad(0.03));
            w.AzimuthJitterStdDev   = FireControlComputer.rad2mil(deg2rad(0.03));
            w.RateOfFire = 85.0; % rounds/min, fully automatic
        end

    end
end
