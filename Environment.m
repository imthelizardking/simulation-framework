classdef Environment < handle
    % ISA (International Standard Atmosphere) model, troposphere + lower
    % stratosphere (valid 0-20 km - comfortably covers every munition in
    % this framework), OR - if loadMetcm() has been called - a measured
    % temperature/pressure/wind profile from a METCM (Computer
    % Meteorological Message, a NATO-style artillery met report) file
    % instead. SeaLevelDensity/SeaLevelSpeedOfSound are the ISA h=0
    % reference values; callers needing the value AT a given altitude use
    % getDensity(h)/getSpeedOfSound(h)/getTemperature(h)/getPressure(h)/
    % getWind(h), not these properties directly. Overriding SeaLevelDensity/
    % SeaLevelSpeedOfSound rescales the ISA altitude curve by ratio rather
    % than being ignored above h=0 - see getDensity/getSpeedOfSound (only
    % applies when no METCM data is loaded).
    properties
        Gravity              (1,1) double = 9.81
        SeaLevelDensity       (1,1) double = 1.225   % kg/m^3, ISA sea-level standard
        SeaLevelSpeedOfSound  (1,1) double = 340.29  % m/s, ISA sea-level standard
        BaseWind_NED          (3,1) double = [0; 0; 0]

        % METCM data (see loadMetcm). Empty MetcmAltitudes = no METCM
        % loaded, ISA formulas and BaseWind_NED are used instead - this is
        % how every getX method below decides which source to use.
        MetcmAltitudes   double = []  % m, breakpoints
        MetcmTemperature double = []  % K, per breakpoint
        MetcmPressure    double = []  % Pa, per breakpoint
        MetcmWind_NED    double = []  % N-by-3 m/s, per breakpoint
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

        function loadMetcm(obj, name)
            % Reads data/<name>.txt (METCM format) and switches this
            % Environment from the ISA formulas to interpolating the
            % file's measured temperature/pressure/wind profile instead.
            % Density and speed of sound are then derived from that
            % measured T/P via the same ideal-gas-law relations used for
            % the ISA fallback - the file doesn't carry them directly.
            % Optional: an Environment that never calls this keeps using
            % ISA. Revert to ISA by clearing MetcmAltitudes ([]).
            %
            % Layer altitudes are NOT encoded in the file; they're this
            % fixed 32-breakpoint table (standard for this message
            % format), matched positionally to the file's data lines in
            % order, after a 2-line message header.
            %
            % Each data line is 16 fixed-width digits, no delimiters:
            %   [2 line index][4 wind dir, mils][2 wind speed, m/s]
            %   [4 temperature, tenths of Kelvin][4 pressure, whole hPa]
            % Temperature/pressure fields are verified against the
            % data/metcm_std.txt example: they match the ISA standard
            % atmosphere to within rounding at every altitude checked. The
            % wind sub-field split (4 digits direction + 2 digits speed)
            % is a reasoned best guess - 4 digits is the minimum that can
            % hold the full 0-6400 mil range - but NOT empirically
            % confirmed, since that example's wind is calm (all-zero)
            % throughout. Check it against a real non-calm METCM file if
            % you have one, and correct here if it's wrong.
            ALTITUDES = [0 100 350 750 1250 1750 2250 2750 3250 3750 4250 4750 ...
                5500 6500 7500 8500 9500 10500 11500 12500 13500 14500 ...
                15500 16500 17500 18500 19500 21000 23000 25000 27000 29000];

            here = fileparts(mfilename('fullpath'));
            path = fullfile(here, 'data', [name '.txt']);
            if ~isfile(path)
                error('Environment:loadMetcm', 'No METCM file found at data/%s.txt', name);
            end

            raw_lines = regexp(fileread(path), '\r\n|\r|\n', 'split');
            raw_lines = raw_lines(~cellfun(@(s) isempty(strtrim(s)), raw_lines));

            n = numel(ALTITUDES);
            if numel(raw_lines) < n + 2
                error('Environment:loadMetcm', ...
                    'data/%s.txt has %d data lines after its 2-line header, expected %d.', ...
                    name, numel(raw_lines) - 2, n);
            end

            T_K   = zeros(n, 1);
            P_Pa  = zeros(n, 1);
            W_NED = zeros(n, 3);

            for i = 1:n
                line = strtrim(raw_lines{i + 2}); % skip the 2-line message header
                wind_dir_mil   = str2double(line(3:6));
                wind_speed_mps = str2double(line(7:8));
                temp_dK        = str2double(line(9:12));
                press_hPa      = str2double(line(13:16));

                T_K(i)  = temp_dK / 10;
                P_Pa(i) = press_hPa * 100; % hPa -> Pa

                % Met convention: direction is where the wind blows FROM,
                % so the velocity vector points the opposite way.
                wind_rad = wind_dir_mil * (2*pi/6400);
                W_NED(i, :) = -wind_speed_mps * [cos(wind_rad), sin(wind_rad), 0];
            end

            obj.MetcmAltitudes   = ALTITUDES(:);
            obj.MetcmTemperature = T_K;
            obj.MetcmPressure    = P_Pa;
            obj.MetcmWind_NED    = W_NED;
        end

        function T = getTemperature(obj, altitude)
            if ~isempty(obj.MetcmAltitudes)
                h = obj.clampToMetcmRange(altitude);
                T = interp1(obj.MetcmAltitudes, obj.MetcmTemperature, h, 'linear');
                return;
            end
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
            if ~isempty(obj.MetcmAltitudes)
                h = obj.clampToMetcmRange(altitude);
                P = interp1(obj.MetcmAltitudes, obj.MetcmPressure, h, 'linear');
                return;
            end
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
            if ~isempty(obj.MetcmAltitudes)
                % Not read from the file directly - derived from the
                % measured temperature/pressure via the ideal gas law.
                T = obj.getTemperature(altitude);
                P = obj.getPressure(altitude);
                Rspecific = obj.Rgas / obj.Mmol;
                rho = P / (Rspecific * T);
                return;
            end
            % SeaLevelDensity scaled by the standard ISA density ratio at
            % this altitude, so an overridden sea-level value still lapses
            % correctly instead of being ignored above h=0.
            rho = obj.SeaLevelDensity * obj.densityRatio(altitude);
        end

        function a = getSpeedOfSound(obj, altitude)
            if ~isempty(obj.MetcmAltitudes)
                T = obj.getTemperature(altitude);
                Rspecific = obj.Rgas / obj.Mmol;
                a = sqrt(obj.GammaAir * Rspecific * T);
                return;
            end
            % SeaLevelSpeedOfSound scaled by sqrt(T(h)/T0) - speed of
            % sound goes as sqrt(temperature).
            a = obj.SeaLevelSpeedOfSound * sqrt(obj.getTemperature(altitude) / obj.T0);
        end

        function w = getWind(obj, altitude)
            % NED wind velocity at altitude. Falls back to the constant
            % BaseWind_NED when no METCM data is loaded. Not currently
            % consumed by any Aero force calculation in this framework -
            % this makes the data available, the same "expose it now,
            % wire it into the physics later" pattern already used for
            % AeroTabulatedSpin's CMA/CMA3 overturning moment.
            if ~isempty(obj.MetcmAltitudes)
                h = obj.clampToMetcmRange(altitude);
                w = [interp1(obj.MetcmAltitudes, obj.MetcmWind_NED(:,1), h, 'linear'); ...
                     interp1(obj.MetcmAltitudes, obj.MetcmWind_NED(:,2), h, 'linear'); ...
                     interp1(obj.MetcmAltitudes, obj.MetcmWind_NED(:,3), h, 'linear')];
                return;
            end
            w = obj.BaseWind_NED;
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

        function h = clampToMetcmRange(obj, altitude)
            % METCM data is measured, not a formula - clamp to the
            % measured range instead of extrapolating past it (same
            % philosophy as BaseAero.evalCoeff's Table clamping).
            h = min(max(altitude, min(obj.MetcmAltitudes)), max(obj.MetcmAltitudes));
        end
    end
end
