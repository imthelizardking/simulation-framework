classdef WeaponPlatform < handle
    % Bundles what's a property of the GUN, not the round: the weapon's
    % own muzzle-velocity ceiling (a chamber-pressure/barrel rating - the
    % ACTUAL velocity for a given shot comes from the munition's Charge,
    % which FireMission checks against this ceiling), mechanical
    % elevation/azimuth traverse limits, per-round aiming jitter, and rate
    % of fire. Feeds FireControlComputer's ElevationMin/Max/AzimuthMin/Max.
    % All angle-valued properties here are in NATO mils (6400/circle, see
    % FireControlComputer.rad2mil/mil2rad) - the launch-angle unit used at
    % every fire-control boundary. This is purely a "launch angle and its
    % peripherals" convention; internal kinematics (Dynamics state, pitch,
    % buildLaunchState's own trig) stay in radians throughout.
    properties
        Name                    (1,:) char   = 'Generic Weapon'
        MaxMuzzleVelocity       (1,1) double = 1000.0  % m/s, weapon's own rated ceiling
        ElevationMin            (1,1) double = FireControlComputer.rad2mil(deg2rad(5.0))
        ElevationMax            (1,1) double = FireControlComputer.rad2mil(deg2rad(85.0))
        AzimuthMin              (1,1) double = -3200   % mils; traverse limit; -3200/3200 = unrestricted (full circle)
        AzimuthMax              (1,1) double = 3200
        ElevationJitterStdDev   (1,1) double = 0.0     % mils, per-round aiming jitter (added on top of the solved angle)
        AzimuthJitterStdDev     (1,1) double = 0.0     % mils, per-round aiming jitter
        RateOfFire              (1,1) double = 60.0    % rounds per minute
    end
end
