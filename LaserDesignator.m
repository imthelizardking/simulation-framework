classdef LaserDesignator < handle
    % An independent scenario entity (ground observer on a mast, or an
    % airborne platform/drone - Position is whatever the runner sets it
    % to) that illuminates a target for a SAL seeker to home on - see
    % SALSeeker, which holds a direct reference to one of these, same
    % "direct object reference" pattern GnssSensor.Jammer uses, rather
    % than routing through Simulation's own (currently unwired -
    % des_bus is built in Simulation.run() but never actually reaches
    % Munition.updateGNC) Designator/PRF_Code hook.
    %
    % Line-of-sight, with no terrain/elevation model in this framework:
    % gated by (1) MaxRange (finite laser power/atmospheric attenuation),
    % and (2) the standard geometric/radio horizon distance, d_km =
    % 3.57*(sqrt(h1_m)+sqrt(h2_m)) - the well-established formula for how
    % far two elevated observers can see each other over the Earth's
    % curvature. This is exactly the real mechanism by which a
    % designator's altitude matters (a mast-mounted ground observer sees
    % much less far than an airborne one at a few thousand meters) even
    % without any actual terrain/obstruction data - it doesn't model a
    % hill blocking the view, but it does model the one geometric LOS
    % limit that's real physics regardless of terrain.
    properties
        Position (3,1) double = [0; 0; 0]  % NED - ground mast or airborne, whatever the runner sets
        MaxRange (1,1) double = 5000.0     % m - effective laser designation range
        PRF_Code (1,1) double = 1688       % illustrative NATO-style pulse-repetition-frequency code
        Active   (1,1) logical = true      % is the designator actively lasing the target right now
    end

    methods
        function obj = LaserDesignator(position, max_range, prf_code)
            if nargin > 0, obj.Position = position; end
            if nargin > 1, obj.MaxRange = max_range; end
            if nargin > 2, obj.PRF_Code = prf_code; end
        end

        function visible = hasLineOfSight(obj, target_pos)
            d = norm(target_pos - obj.Position);
            if d > obj.MaxRange
                visible = false;
                return;
            end
            h1 = max(CoordinateFrame.altitude(obj.Position, 'NED'), 0);
            h2 = max(CoordinateFrame.altitude(target_pos, 'NED'), 0);
            horizon_km = 3.57 * (sqrt(h1) + sqrt(h2));
            visible = (d / 1000) <= horizon_km;
        end

        function is_lit = illuminates(obj, target_pos)
            is_lit = obj.Active && obj.hasLineOfSight(target_pos);
        end
    end
end
