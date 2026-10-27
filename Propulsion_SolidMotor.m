classdef Propulsion_SolidMotor < BasePropulsion
    % Solid rocket motor, thrust along body +x. Two ways to define it:
    %
    %   Method 1 - average thrust + burn time (constant-thrust step):
    %       motor = Propulsion_SolidMotor(avg_thrust_N, burn_time_s)
    %       motor = Propulsion_SolidMotor(avg_thrust_N, burn_time_s, propellant_kg)
    %
    %   Method 2 - time vs thrust table (linear interpolation, 0 outside
    %   the table's time span):
    %       motor = Propulsion_SolidMotor([t_s(:), F_N(:)])
    %       motor = Propulsion_SolidMotor([t_s(:), F_N(:)], propellant_kg)
    %       motor = Propulsion_SolidMotor(t_s, F_N, propellant_kg)
    %
    % The constructor tells the two apart by shape: two scalars = method
    % 1, an Nx2 matrix or two equal-length vectors = method 2. Explicit
    % fromAverage/fromTable factories are also provided, plus fromData for
    % the MunitionFactory data-override path.
    %
    % Mass flow: with propellant mass (inherited FuelMass) > 0, mass is
    % consumed in proportion to delivered impulse - m_dot = -Mp*F(t)/I_tot
    % - so exactly Mp is burned by burnout and the burn rate follows the
    % thrust curve (constant Isp assumption). Method 1 reduces to the
    % constant -Mp/t_b. With FuelMass = 0 (the default) m_dot stays 0, the
    % same constant-mass behavior this class had before propellant mass
    % was supported. Munition's shared evaluator owns current mass/CG/inertia;
    % massConsumed supplies exact cumulative consumption for state projection.
    %
    % No ambient-pressure correction - the table/average thrust is used
    % as-is at every altitude.
    properties
        Name         (1,:) char   = ''        % optional motor designation, e.g. 'MK66 Mod4'
        Mode         (1,:) char   = 'average' % 'average' | 'table'
        ThrustForce  (1,1) double = 1200.0    % N - method 1 input; for a table, its impulse-averaged thrust (info only)
        BurnDuration (1,1) double = 2.0       % s - method 1 input; for a table, its last time point
        TimeTable    (:,1) double = []        % s   - method 2 only
        ThrustTable  (:,1) double = []        % N   - method 2 only
    end

    properties (SetAccess = private)
        TotalImpulse (1,1) double = 0 % N*s
    end

    methods
        function obj = Propulsion_SolidMotor(varargin)
            if nargin == 0
                obj.TotalImpulse = obj.ThrustForce * obj.BurnDuration;
                return;
            end

            a1 = varargin{1};
            if isscalar(a1)
                % Method 1: (avg_thrust, burn_time [, propellant_mass])
                if nargin < 2
                    error('Propulsion_SolidMotor:args', ...
                        'Average-thrust mode needs both avg_thrust and burn_time.');
                end
                obj.setAverage(a1, varargin{2});
                if nargin > 2, obj.FuelMass = varargin{3}; end
            elseif size(a1, 2) == 2 && size(a1, 1) >= 2
                % Method 2: (table_Nx2 [, propellant_mass])
                obj.setTable(a1(:,1), a1(:,2));
                if nargin > 1, obj.FuelMass = varargin{2}; end
            elseif nargin >= 2 && isvector(a1) && numel(a1) == numel(varargin{2})
                % Method 2: (t_vec, F_vec [, propellant_mass])
                obj.setTable(a1, varargin{2});
                if nargin > 2, obj.FuelMass = varargin{3}; end
            else
                error('Propulsion_SolidMotor:args', ...
                    ['Expected (avg_thrust, burn_time[, Mp]), (table_Nx2[, Mp]) ' ...
                     'or (t_vec, F_vec[, Mp]).']);
            end

            if obj.FuelMass < 0
                error('Propulsion_SolidMotor:args', 'Propellant mass must be >= 0.');
            end
        end

        function setAverage(obj, avg_thrust, burn_time)
            if ~(isscalar(avg_thrust) && avg_thrust >= 0)
                error('Propulsion_SolidMotor:args', 'avg_thrust must be a scalar >= 0.');
            end
            if ~(isscalar(burn_time) && burn_time > 0)
                error('Propulsion_SolidMotor:args', 'burn_time must be a scalar > 0.');
            end
            obj.Mode         = 'average';
            obj.ThrustForce  = avg_thrust;
            obj.BurnDuration = burn_time;
            obj.TimeTable    = [];
            obj.ThrustTable  = [];
            obj.TotalImpulse = avg_thrust * burn_time;
        end

        function setTable(obj, t, F)
            t = double(t(:));
            F = double(F(:));
            if numel(t) < 2 || numel(t) ~= numel(F)
                error('Propulsion_SolidMotor:table', ...
                    'Thrust table needs >= 2 points and equal-length time/thrust columns.');
            end
            if any(~isfinite(t)) || any(~isfinite(F))
                error('Propulsion_SolidMotor:table', 'Thrust table contains NaN/Inf.');
            end
            if t(1) < 0 || any(diff(t) <= 0)
                error('Propulsion_SolidMotor:table', ...
                    'Table time must start at >= 0 and be strictly increasing.');
            end
            if any(F < 0)
                error('Propulsion_SolidMotor:table', 'Table thrust must be >= 0.');
            end
            obj.Mode         = 'table';
            obj.TimeTable    = t;
            obj.ThrustTable  = F;
            obj.BurnDuration = t(end);
            obj.TotalImpulse = trapz(t, F);
            obj.ThrustForce  = obj.TotalImpulse / (t(end) - t(1));
        end

        function F = thrustMagnitude(obj, t)
            % Scalar thrust (N) at time t since launch.
            switch obj.Mode
                case 'table'
                    if t < obj.TimeTable(1) || t > obj.TimeTable(end)
                        F = 0.0;
                    else
                        F = interp1(obj.TimeTable, obj.ThrustTable, t, 'linear');
                    end
                otherwise
                    if t >= 0 && t <= obj.BurnDuration
                        F = obj.ThrustForce;
                    else
                        F = 0.0;
                    end
            end
        end

        function consumed = massConsumed(obj, t)
            % Exact cumulative table/constant flow; avoids RK4 drift across
            % the burn boundary. Pure lookup, safe for repeated substeps.
            consumed = 0;
            if obj.FuelMass == 0 || obj.TotalImpulse <= 0, return; end
            if strcmp(obj.Mode, 'average')
                fraction = min(1, max(0, t / obj.BurnDuration));
            elseif t <= obj.TimeTable(1)
                fraction = 0;
            elseif t >= obj.TimeTable(end)
                fraction = 1;
            else
                before = obj.TimeTable < t;
                times = [obj.TimeTable(before); t];
                thrust = [obj.ThrustTable(before); interp1(obj.TimeTable, obj.ThrustTable, t)];
                fraction = trapz(times, thrust) / obj.TotalImpulse;
            end
            consumed = obj.FuelMass * min(1, max(0, fraction));
        end

        function [F_thrust, M_torque, m_dot] = getThrust(obj, t, ~)
            F = obj.thrustMagnitude(t);
            F_thrust = [F; 0; 0];
            M_torque = [0; 0; 0];
            if obj.FuelMass > 0 && obj.TotalImpulse > 0
                m_dot = -obj.FuelMass * F / obj.TotalImpulse;
            else
                m_dot = 0.0;
            end
        end
    end

    methods (Static)
        function obj = fromAverage(avg_thrust, burn_time, propellant_mass)
            if nargin < 3, propellant_mass = 0; end
            obj = Propulsion_SolidMotor(avg_thrust, burn_time, propellant_mass);
        end

        function obj = fromTable(t, F, propellant_mass)
            if nargin < 3, propellant_mass = 0; end
            obj = Propulsion_SolidMotor(t, F, propellant_mass);
        end

        function obj = MK66Mod4(propellant_mass)
            % Mk 66 Mod 4 (Hydra 70 / APKWS) rocket motor, as a thrust
            % table. Digitized by eye from a published "Thrust Curve
            % Comparison" chart (MOD 4 trace, lbf vs s, ~70 degF static
            % test) - shape and impulse are representative, individual
            % points are only good to roughly +/-50 lbf / +/-5 ms. The
            % ~70-80 lbf baseline before ignition and after burnout is
            % kept as read from the chart (likely load-cell offset); the
            % table ends at 1.20 s, after which thrust is 0.
            if nargin < 1, propellant_mass = 0; end
            LBF_TO_N = 4.4482216;
            tbl_lbf = [ ...
                0.000,   80
                0.012,   90
                0.015,  400
                0.021, 1100
                0.027, 1700
                0.034, 1980
                0.052, 1800
                0.075, 1720
                0.092, 1680
                0.134, 1600
                0.176, 1560
                0.210, 1540
                0.268, 1500
                0.294, 1470
                0.319, 1470
                0.336, 1500
                0.352, 1530
                0.378, 1510
                0.403, 1550
                0.436, 1540
                0.453, 1580
                0.468, 1630
                0.495, 1590
                0.520, 1600
                0.545, 1620
                0.570, 1680
                0.600, 1690
                0.629, 1780
                0.663, 1850
                0.696, 1880
                0.747, 1940
                0.789, 1990
                0.830, 2040
                0.864, 2080
                0.885, 2100
                0.902, 2060
                0.919, 1900
                0.938, 1500
                0.956, 1000
                0.973,  500
                0.990,  250
                1.013,  100
                1.040,  130
                1.061,   80
                1.124,   75
                1.200,   70 ];
            obj = Propulsion_SolidMotor([tbl_lbf(:,1), tbl_lbf(:,2) * LBF_TO_N], ...
                propellant_mass);
            obj.Name = 'MK66 Mod4';
        end

        function obj = fromData(data, default_thrust, default_burn_time)
            % MunitionFactory data-override hook. Recognized fields:
            %   ThrustTable    - Nx2 [time_s, thrust_N] -> method 2 (wins if present)
            %   Thrust         - average thrust, N      -> method 1
            %   BurnTime       - burn time, s           -> method 1
            %   PropellantMass - kg, optional (enables mass depletion)
            % Missing fields fall back to the factory's defaults, so a
            % munition with no data file behaves exactly as before.
            %
            % default_thrust may instead be a Propulsion_SolidMotor (e.g.
            % MK66Mod4()), used as-is when the data file has no motor
            % fields; Thrust/BurnTime given without the other fall back to
            % that motor's impulse-averaged thrust / burn duration.
            mp  = FactoryData.getOr(data, 'PropellantMass', 0);
            tbl = FactoryData.getOr(data, 'ThrustTable', []);
            if isa(default_thrust, 'Propulsion_SolidMotor')
                base = default_thrust;
                default_thrust    = base.ThrustForce;
                default_burn_time = base.BurnDuration;
                has_avg = isstruct(data) && (isfield(data, 'Thrust') || isfield(data, 'BurnTime'));
                if isempty(tbl) && ~has_avg
                    obj = base;
                    obj.FuelMass = FactoryData.getOr(data, 'PropellantMass', base.FuelMass);
                    return;
                end
            end
            if ~isempty(tbl)
                obj = Propulsion_SolidMotor(tbl, mp);
            else
                obj = Propulsion_SolidMotor( ...
                    FactoryData.getOr(data, 'Thrust', default_thrust), ...
                    FactoryData.getOr(data, 'BurnTime', default_burn_time), mp);
            end
        end
    end
end
