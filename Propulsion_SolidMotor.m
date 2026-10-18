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
    % was supported. Inertia/CG are NOT updated here - MassProperties.
    % Inertia stays at its launch value for the whole flight.
    %
    % No ambient-pressure correction - the table/average thrust is used
    % as-is at every altitude.
    properties
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

        function obj = fromData(data, default_thrust, default_burn_time)
            % MunitionFactory data-override hook. Recognized fields:
            %   ThrustTable    - Nx2 [time_s, thrust_N] -> method 2 (wins if present)
            %   Thrust         - average thrust, N      -> method 1
            %   BurnTime       - burn time, s           -> method 1
            %   PropellantMass - kg, optional (enables mass depletion)
            % Missing fields fall back to the factory's defaults, so a
            % munition with no data file behaves exactly as before.
            mp  = FactoryData.getOr(data, 'PropellantMass', 0);
            tbl = FactoryData.getOr(data, 'ThrustTable', []);
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
