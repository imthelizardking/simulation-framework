%% File: TargetBase.m
classdef (Abstract) TargetBase < handle
    properties
        Name              (1,:) char   = 'Target'
        Position          (3,1) double = [0; 0; 0]
        Velocity          (3,1) double = [0; 0; 0]
        Acceleration      (3,1) double = [0; 0; 0]
        LaserReflectivity (1,1) double = 0.3    % Diffuse reflectance [0.0 to 1.0]
        ThermalSignature  (1,1) double = 350.0  % Radiative temp (K)
        RCS               (1,1) double = 1.0    % Radar cross section (m^2)
    end
    
    methods
        function obj = TargetBase(name, pos, vel)
            if nargin > 0, obj.Name = name; end
            if nargin > 1, obj.Position = pos; end
            if nargin > 2, obj.Velocity = vel; end
        end
        
        function bus = getTargetBus(obj)
            bus = struct(...
                'pos',               obj.Position, ...
                'vel',               obj.Velocity, ...
                'accel',             obj.Acceleration, ...
                'LaserReflectivity', obj.LaserReflectivity, ...
                'ThermalSignature',  obj.ThermalSignature, ...
                'RCS',               obj.RCS);
        end
    end
    
    methods (Abstract)
        step(obj, t, dt, munition_truth)
    end
end