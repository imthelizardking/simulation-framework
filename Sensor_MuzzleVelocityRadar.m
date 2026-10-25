classdef Sensor_MuzzleVelocityRadar < BaseSensor
    % Course-correction fuze sensor: reports the round's true muzzle
    % (launch) velocity, once, after AcquisitionDelay seconds of flight -
    % not a continuous readout. Deliberately simplified: a real muzzle-
    % velocity radar back-solves launch speed from several early-flight
    % range/time observations; this class instead reads the true launch
    % speed directly (TrueMuzzleVelocity, wired up by the runner at
    % launch time - see runner_25mm_course_correction.m) and adds a
    % single Gaussian perturbation to it, standing in for that radar's
    % own measurement error.
    properties
        AcquisitionDelay   (1,1) double = 1.0   % s since launch before the first (only) reading is available
        NoiseStdDev        (1,1) double = 3.0   % m/s, 1-sigma perturbation applied to the single reading
        TrueMuzzleVelocity (1,1) double = 0.0   % m/s - set externally by the runner to the round's actual (dispersed) launch speed
    end

    properties (Access = private)
        HasFired      (1,1) logical = false
        CachedReading struct = struct('Valid', false, 'MuzzleVelocity', NaN)
    end

    methods
        function meas = step(obj, t, ~, ~, ~)
            if obj.HasFired
                meas = obj.CachedReading;
                return;
            end
            if t < obj.AcquisitionDelay
                meas = struct('Valid', false, 'MuzzleVelocity', NaN);
                return;
            end
            measured = obj.TrueMuzzleVelocity + obj.NoiseStdDev * randn();
            obj.CachedReading = struct('Valid', true, 'MuzzleVelocity', measured);
            obj.HasFired = true;
            meas = obj.CachedReading;
        end
    end
end
