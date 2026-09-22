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
        function obj = TargetBase(name, pos, vel, frame)
            % pos/vel are always stored in the framework's Z-Up-sense
            % convention (downrange, crossrange, altitude/climb-rate) -
            % the same "author once, auto-rotated at the point of
            % consumption" contract Simulation.resolveTargetBus/
            % ParticleSwarmSimulation.resolveTargetBus already document.
            % frame (optional, default 'Z-Up') lets a caller author pos/
            % vel directly in a munition's own convention instead (e.g.
            % 'FDR' - forward/down/right, matching DynamicsMPMM/AeroMPMM)
            % when that's more natural for the scenario being built -
            % converted ONCE here via the same self-inverse
            % CoordinateFrame.rotateVector every Dynamics class already
            % uses, so Position/Velocity/getTargetBus and everything
            % downstream never has to know which convention the scenario
            % was actually written in.
            if nargin > 0, obj.Name = name; end
            if nargin > 1
                if nargin > 3 && ~isempty(frame)
                    pos = CoordinateFrame.rotateVector(pos, frame);
                end
                obj.Position = pos;
            end
            if nargin > 2
                if nargin > 3 && ~isempty(frame)
                    vel = CoordinateFrame.rotateVector(vel, frame);
                end
                obj.Velocity = vel;
            end
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