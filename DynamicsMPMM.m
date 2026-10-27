classdef DynamicsMPMM < BaseDynamics
    % High-fidelity STANAG 4355-style Modified Point-Mass Model dynamics:
    % real (inverse-square, latitude-corrected) gravity, optional Earth-
    % rotation Coriolis acceleration, and spin integration - ported from a
    % reference calc_accs.m implementation (see example_code.m). Position/
    % velocity/spin (IDX_BODYRATE(1)) are integrated; IDX_EULER stays
    % algebraic (its derivative is held at zero) - a full attitude state
    % isn't needed here because the yaw-of-repose angle that would drive
    % it is already handled as a quasi-steady quantity by the paired
    % AeroMPMM (see that class - it, not this one, computes alpha_e/
    % drag/lift/Magnus/spin-damping-moment; this class only integrates
    % what AeroMPMM/Propulsion hand it, plus gravity and Coriolis, which
    % need no atmosphere lookup and so belong here rather than in an Aero
    % class - see computeGravityVector/computeCoriolisVector).
    % mass_prop.Inertia is expected in the same diag([Ix,Iy,Iz])
    % convention MunitionFactory already uses elsewhere.
    %
    % Always FDR (X=forward, Y=down, Z=right - see CoordinateFrame.m):
    % unlike Dynamics3Dof's dual Z-Up/NED Frame toggle, the 17-state
    % blueprint layout this class uses (BaseDynamics.IDX_POS et al.) is
    % documented FDR-only, so there's no Frame property here to keep in
    % sync - only AeroMPMM.LatitudeDeg needs to match this class's own
    % InitLatitudeDeg. Note FDR is left-handed (see CoordinateFrame.m),
    % so every genuine cross-product term below (Coriolis) carries an
    % extra sign flip relative to a straight NED port - see
    % computeCoriolisVector.
    properties
        % Launch latitude (degrees, + north/- south) - feeds the real-
        % gravity latitude correction and the Coriolis term. Must match
        % the paired AeroMPMM's LatitudeDeg (gravity feeds its alpha_e
        % solve too - see AeroMPMM.getForcesAndMoments).
        InitLatitudeDeg (1,1) double = 0.0

        % True compass bearing (radians, 0 = North, clockwise-positive)
        % of this Dynamics' own local X (forward/downrange) axis - NOT
        % the same thing as a shot's aim azimuth (which rotates the
        % launch velocity WITHIN this local frame, see buildLaunchState
        % elsewhere in this framework). Only used to project Earth's
        % rotation vector onto the local frame for Coriolis; leave at the
        % default 0 (X assumed to point true North) unless you actually
        % know the local frame's real heading.
        InitAzimuth (1,1) double = 0.0

        % Earth-rotation Coriolis acceleration on/off - matches
        % example_code.m's own SIM_CORIOLIS toggle.
        SimCoriolis (1,1) logical = true
    end

    properties (Constant)
        G0                 = 9.80665     % m/s^2, mean sea-level gravity (latitude-corrected below)
        EARTH_RADIUS       = 6356766.0   % m, mean Earth radius used in the inverse-square gravity term
        EARTH_ROTATION_RATE = 7.292115e-5 % rad/s, Earth's sidereal rotation rate
    end

    methods
        function obj = DynamicsMPMM()
            obj.StateDimension = BaseDynamics.FULL_STATE_DIM;
        end

        function X0 = buildLaunchState(obj, launcher_pos, v0, elevation, azimuth, mass, spin0)
            % Assembles the 17-element state from firing angles and
            % launch speed, same "elevation/azimuth keep their familiar
            % Z-Up meaning regardless of frame" contract Dynamics3Dof
            % uses - this class is always FDR (see class comment), so the
            % rotation is applied directly rather than via an obj.Frame
            % toggle. IDX_EULER/IDX_INERTIA are set once for the record
            % (telemetry/diagnostics) but never integrated/read back by
            % computeDerivatives - see this file's class comment.
            if nargin < 7, spin0 = 0.0; end
            v_x = v0 * cos(elevation) * cos(azimuth);
            v_y = v0 * cos(elevation) * sin(azimuth);
            v_z = v0 * sin(elevation);
            pos   = CoordinateFrame.rotateVector(launcher_pos, 'FDR');
            vel   = CoordinateFrame.rotateVector([v_x; v_y; v_z], 'FDR');
            pitch = CoordinateFrame.rotateAngle(elevation, 'FDR');

            X0 = zeros(BaseDynamics.FULL_STATE_DIM, 1);
            X0(BaseDynamics.IDX_POS)         = pos;
            X0(BaseDynamics.IDX_VEL)         = vel;
            X0(BaseDynamics.IDX_EULER)       = [0; pitch; azimuth];
            X0(BaseDynamics.IDX_BODYRATE(1)) = spin0;
            X0(BaseDynamics.IDX_MASS)        = mass;
        end

        function dX = computeDerivatives(obj, t, X, forces, moments, mass_prop, m_dot)
            m   = X(BaseDynamics.IDX_MASS);
            Ix  = mass_prop.Inertia(1,1);
            pos = X(BaseDynamics.IDX_POS);
            vel = X(BaseDynamics.IDX_VEL);

            acc_gravity  = DynamicsMPMM.computeGravityVector(pos, obj.InitLatitudeDeg);
            acc_coriolis = DynamicsMPMM.computeCoriolisVector(vel, obj.InitLatitudeDeg, obj.InitAzimuth, obj.SimCoriolis);

            F_total = forces.Aero + forces.Propulsion + m * (acc_gravity + acc_coriolis);

            dX = zeros(BaseDynamics.FULL_STATE_DIM, 1);
            dX(BaseDynamics.IDX_POS)         = vel;
            dX(BaseDynamics.IDX_VEL)         = F_total / m;
            dX(BaseDynamics.IDX_BODYRATE(1)) = moments.Aero(1) / Ix;
            dX(BaseDynamics.IDX_MASS)        = m_dot;
        end
    end

    methods (Static)
        function acc_gravity = computeGravityVector(pos, latitude_deg)
            % Inverse-square gravity toward Earth's center, with the
            % standard latitude correction for Earth's oblateness
            % (example_code.m's "method 3"). pos is this Dynamics'
            % FDR position (X=forward, Y=down, Z=right), so Earth's
            % center sits at [0;EARTH_RADIUS;0] from the launch point -
            % NOT [0;-R;0] as in example_code.m's own Y-up frame, and not
            % [0;0;R] as under the (unused here) NED convention; see this
            % file's class comment. No cross product is involved, so this
            % is a pure index relabeling - the handedness flip that FDR
            % carries relative to NED doesn't apply here.
            g0 = DynamicsMPMM.G0;
            g  = g0 * (1 - 0.0026 * cosd(2 * latitude_deg));
            R  = DynamicsMPMM.EARTH_RADIUS;
            r  = pos - [0; R; 0];
            r_mag = norm(r);
            acc_gravity = -g * (R^2 / r_mag^3) * r;
        end

        function acc_coriolis = computeCoriolisVector(vel, latitude_deg, azimuth, sim_coriolis)
            % Earth-rotation Coriolis acceleration, -2*(Omega x vel).
            % Omega (Earth's rotation vector) is expressed in this
            % Dynamics' local FDR frame from latitude and the frame's own
            % true bearing (azimuth, see InitAzimuth):
            % v_Omega = [Ohm*cosd(LAT)*cosd(AZ); -Ohm*sind(LAT);
            % -Ohm*cosd(LAT)*sind(AZ)] - same physical components as the
            % NED derivation, just with the down/right entries swapped to
            % match FDR's Y=down/Z=right (see this file's class comment).
            % FDR is left-handed, so the cross product itself also flips
            % sign relative to the NED formula (-2*cross(...) becomes
            % +2*cross(...)) to represent the same physical acceleration -
            % verified independently against the raw (forward,right,down)
            % vector algebra, not just the abstract sign rule.
            if nargin < 4 || ~sim_coriolis
                acc_coriolis = [0; 0; 0];
                return;
            end
            Omega = DynamicsMPMM.EARTH_ROTATION_RATE;
            v_omega = Omega * [cosd(latitude_deg) * cos(azimuth); ...
                               -sind(latitude_deg); ...
                               -cosd(latitude_deg) * sin(azimuth)];
            acc_coriolis = 2 * cross(v_omega, vel);
        end
    end
end
