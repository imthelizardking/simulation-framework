classdef WeaponPlatform < handle
    % Bundles what's a property of the GUN, not the round: muzzle
    % velocity (nominal + round-to-round variation), mechanical elevation/
    % azimuth traverse limits, per-round aiming jitter, and rate of fire.
    % Feeds FireControlComputer's ElevationMin/Max/AzimuthMin/Max and a
    % FireMission's dispersion sampling.
    properties
        Name                    (1,:) char   = 'Generic Weapon'
        MuzzleVelocity          (1,1) double = 800.0   % m/s, nominal
        MuzzleVelocityStdDev    (1,1) double = 0.0     % m/s, round-to-round variation
        ElevationMin            (1,1) double = deg2rad(5.0)
        ElevationMax            (1,1) double = deg2rad(85.0)
        AzimuthMin              (1,1) double = -pi     % traverse limit; -pi/pi = unrestricted
        AzimuthMax              (1,1) double = pi
        ElevationJitterStdDev   (1,1) double = 0.0     % rad, per-round aiming jitter (added on top of the solved angle)
        AzimuthJitterStdDev     (1,1) double = 0.0     % rad, per-round aiming jitter
        RateOfFire              (1,1) double = 60.0    % rounds per minute
    end
end
