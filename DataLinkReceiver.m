%% File: DataLinkReceiver.m
classdef DataLinkReceiver < BaseSensor
    properties
        BitErrorRate (1,1) double = 0.0
        PacketLoss   (1,1) double = 0.01 % 1% dropped packets
    end
    
    methods
        function obj = DataLinkReceiver(rate)
            if nargin < 1, rate = 50; end % 50 Hz uplink
            obj@BaseSensor('DataLink', rate);
        end
    end
    
    methods (Access = protected)
        function meas_bus = generateMeasurement(obj, ~, ~, ~, world_truth)
            % Reads uplink from ground station passed in world_truth
            if ~isfield(world_truth, 'GroundUplink') || ~world_truth.GroundUplink.Valid
                meas_bus = struct('Valid', false, 'AccelCmd', [0;0;0]);
                return;
            end
            
            % Simulate packet loss
            if rand() < obj.PacketLoss
                meas_bus = struct('Valid', false, 'AccelCmd', [0;0;0]);
                return;
            end
            
            meas_bus = world_truth.GroundUplink;
        end
    end
end