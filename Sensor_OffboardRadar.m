%% File: Sensor_OffboardRadar.m
classdef Sensor_OffboardRadar < handle
    properties
        Position          (3,1) double = [0; 0; 0]   % Radar site location (NED)
        MaxRange          (1,1) double = 25000.0     % Radar instrumented range (m)
        BeamWidth         (1,1) double = deg2rad(1.5)% Tracking beamwidth
        RangeNoiseStd     (1,1) double = 5.0         % Range noise (m)
        AngleNoiseStd     (1,1) double = deg2rad(0.05)% Tracking jitter (rad)
    end
    
    methods
        function obj = Sensor_OffboardRadar(site_pos)
            if nargin > 0, obj.Position = site_pos; end
        end
        
        function track_data = trackObject(obj, true_pos)
            r_vec = true_pos - obj.Position;
            dist  = norm(r_vec);
            
            if dist > obj.MaxRange
                track_data = struct('Valid', false, 'Range', NaN, 'Azimuth', NaN, 'Elevation', NaN);
                return;
            end
            
            % Add tracking noise
            noisy_dist = dist + obj.RangeNoiseStd * randn();
            az = atan2(r_vec(2), r_vec(1)) + obj.AngleNoiseStd * randn();
            el = atan2(-r_vec(3), sqrt(r_vec(1)^2 + r_vec(2)^2)) + obj.AngleNoiseStd * randn();
            
            track_data = struct('Valid', true, 'Range', noisy_dist, 'Azimuth', az, 'Elevation', el, ...
                                'EstimatedPos', obj.Position + noisy_dist * [cos(el)*cos(az); cos(el)*sin(az); -sin(el)]);
        end
    end
end