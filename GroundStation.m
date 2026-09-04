%% File: GroundStation.m
classdef GroundStation < handle
    properties
        Position         (3,1) double = [0; 0; 0]
        Radar            OffboardRadar
        GuidanceMode     (1,:) char = 'CLOS' % Command Line-of-Sight
    end
    
    methods
        function obj = GroundStation(site_pos)
            if nargin > 0, obj.Position = site_pos; end
            obj.Radar = OffboardRadar(obj.Position);
        end
        
        function uplink_bus = step(obj, t, dt, msl_pos, tgt_pos)
            % 1. Radar tracks both missile and target
            tgt_track = obj.Radar.trackObject(tgt_pos);
            msl_track = obj.Radar.trackObject(msl_pos);
            
            if ~tgt_track.Valid || ~msl_track.Valid
                uplink_bus = struct('Valid', false, 'AccelCmd', [0; 0; 0], 'TargetPosEst', [NaN; NaN; NaN]);
                return;
            end
            
            % 2. Command Line-of-Sight (CLOS) Guidance:
            % Keeps missile centered on the Ground-to-Target sightline
            r_tgt = tgt_track.EstimatedPos - obj.Position;
            los_unit = r_tgt / norm(r_tgt);
            
            % Perpendicular deviation of missile from the radar-target line
            r_msl = msl_track.EstimatedPos - obj.Position;
            cross_track_error = r_msl - dot(r_msl, los_unit) * los_unit;
            
            % Ground guidance computes steering command to drive error to zero
            k_p = 4.0;
            accel_cmd_ground = -k_p * cross_track_error;
            
            uplink_bus = struct(...
                'Valid',        true, ...
                'AccelCmd',     accel_cmd_ground, ...
                'TargetPosEst', tgt_track.EstimatedPos, ...
                'Timestamp',    t);
        end
    end
end