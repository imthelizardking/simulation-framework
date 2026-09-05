classdef Environment < handle
    % ISA (International Standard Atmosphere) model, troposphere + lower
    % stratosphere (valid 0-20 km - comfortably covers every munition in
    % this framework). SeaLevelDensity/SeaLevelSpeedOfSound are the h=0
    % reference values; callers needing the value AT a given altitude use
    % getDensity(h)/getSpeedOfSound(h)/getTemperature(h)/getPressure(h),
    % not these properties directly. Overriding SeaLevelDensity/
    % SeaLevelSpeedOfSound rescales the altitude curve by ratio rather
    % than being ignored above h=0 - see getDensity/getSpeedOfSound.
    properties
        Gravity              (1,1) double = 9.81
        SeaLevelDensity       (1,1) double = 1.225   % kg/m^3, ISA sea-level standard
        SeaLevelSpeedOfSound  (1,1) double = 340.29  % m/s, ISA sea-level standard
        BaseWind_NED          (3,1) double = [0; 0; 0]
    end

    properties (Constant, Access = private)
        T0      = 288.15   % K, ISA sea-level standard temperature
        L       = 0.0065   % K/m, tropospheric lapse rate (0-11 km)
        H_TROP  = 11000.0  % m, top of the troposphere layer
        T_STRAT = 216.65   % K, isothermal stratosphere temperature (11-20 km)
        G0      = 9.80665  % m/s^2, standard gravity used in the barometric
                            % formulas below - distinct from Gravity, which
                            % drives the actual vehicle dynamics and may be
                            % overridden independently of this ISA model
        Mmol     = 0.0289644 % kg/mol, molar mass of air
        Rgas     = 8.31447   % J/(mol*K), universal gas constant
        GammaAir = 1.4       % ratio of specific heats for air
    end

    methods
        function obj = Environment(density, gravity, wind, speed_of_sound)
            if nargin > 0, obj.SeaLevelDensity = density; end
            if nargin > 1, obj.Gravity = gravity; end
            if nargin > 2, obj.BaseWind_NED = wind; end
            if nargin > 3, obj.SeaLevelSpeedOfSound = speed_of_sound; end
        end

        function T = getTemperature(obj, altitude)
            % Standard ISA lapse shape - not rescaled by SeaLevelDensity/
            % SeaLevelSpeedOfSound, which only rescale density/speed-of-
            % sound magnitude (see getDensity/getSpeedOfSound).
            h = max(altitude, 0);
            if h <= obj.H_TROP
                T = obj.T0 - obj.L * h;
            else
                T = obj.T_STRAT;
            end
        end

        function P = getPressure(obj, altitude)
            h = max(altitude, 0);
            P0 = obj.seaLevelPressure();
            expo = (obj.G0 * obj.Mmol) / (obj.Rgas * obj.L);
            if h <= obj.H_TROP
                T = obj.getTemperature(h);
                P = P0 * (T / obj.T0)^expo;
            else
                P11 = P0 * (obj.T_STRAT / obj.T0)^expo;
                P = P11 * exp(-obj.G0 * obj.Mmol * (h - obj.H_TROP) / (obj.Rgas * obj.T_STRAT));
            end
        end

        function rho = getDensity(obj, altitude)
            % SeaLevelDensity scaled by the standard ISA density ratio at
            % this altitude, so an overridden sea-level value still lapses
            % correctly instead of being ignored above h=0.
            rho = obj.SeaLevelDensity * obj.densityRatio(altitude);
        end

        function a = getSpeedOfSound(obj, altitude)
            % SeaLevelSpeedOfSound scaled by sqrt(T(h)/T0) - speed of
            % sound goes as sqrt(temperature).
            a = obj.SeaLevelSpeedOfSound * sqrt(obj.getTemperature(altitude) / obj.T0);
        end
    end

    methods (Access = private)
        function P0 = seaLevelPressure(obj)
            Rspecific = obj.Rgas / obj.Mmol;
            P0 = obj.SeaLevelDensity * Rspecific * obj.T0;
        end

        function ratio = densityRatio(obj, altitude)
            h = max(altitude, 0);
            expo = (obj.G0 * obj.Mmol) / (obj.Rgas * obj.L) - 1; % rho ~ T^expo in the troposphere
            if h <= obj.H_TROP
                T = obj.getTemperature(h);
                ratio = (T / obj.T0)^expo;
            else
                ratio11 = (obj.T_STRAT / obj.T0)^expo;
                ratio = ratio11 * exp(-obj.G0 * obj.Mmol * (h - obj.H_TROP) / (obj.Rgas * obj.T_STRAT));
            end
        end
    end
end
