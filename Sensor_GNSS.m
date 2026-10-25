classdef GnssSensor < BaseSensor
    % GNSS position/velocity fix, at a realistic update rate (not every
    % physics step - a real GPS receiver outputs at ~1-10 Hz, far slower
    % than a ~500 Hz-1 kHz IMU) - callers (e.g. a midcourse guidance's
    % dead-reckoning INS) must handle Valid=false between fixes rather
    % than expecting a fresh reading every call.
    %
    % Jamming: a spatial denial zone via the optional Jammer property
    % (a Jammer object - Position + EffectiveRadius). Empty (default) =
    % no jamming modeled at all, not "jamming disabled" as a separate
    % flag - matching this framework's existing "optional collaborator,
    % [] = not present" convention (e.g. Munition.Propulsion/Guidance).
    properties
        UpdateRateHz (1,1) double = 5.0   % Hz - realistic GPS fix rate
        PosNoiseStd  (1,1) double = 5.0   % m, 1-sigma per-axis position noise
        VelNoiseStd  (1,1) double = 0.5   % m/s, 1-sigma per-axis velocity noise
        Jammer = []                        % optional Jammer object - see class comment
    end

    properties (Access = private)
        LastFixTime (1,1) double = -Inf
    end

    methods
        function obj = GnssSensor()
            obj.Name = 'GNSS';
        end

        function meas = step(obj, t, truth_state, ~, ~)
            pos = truth_state(BaseDynamics.IDX_POS);

            if ~isempty(obj.Jammer) && norm(pos - obj.Jammer.Position) <= obj.Jammer.EffectiveRadius
                meas = struct('Valid', false, 'Position', [NaN;NaN;NaN], 'Velocity', [NaN;NaN;NaN]);
                return;
            end

            if (t - obj.LastFixTime) < (1.0 / obj.UpdateRateHz)
                meas = struct('Valid', false, 'Position', [NaN;NaN;NaN], 'Velocity', [NaN;NaN;NaN]);
                return;
            end
            obj.LastFixTime = t;

            vel = truth_state(BaseDynamics.IDX_VEL);
            meas = struct('Valid', true, ...
                'Position', pos + obj.PosNoiseStd * randn(3,1), ...
                'Velocity', vel + obj.VelNoiseStd * randn(3,1));
        end
    end
end
