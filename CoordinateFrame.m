%% File: CoordinateFrame.m
classdef CoordinateFrame
    % Shared conversion between the framework's three supported position/
    % velocity conventions:
    %   'Z-Up' (default) - the framework's original convention, unchanged.
    %       X = downrange, Y = crossrange, Z = up.
    %   'NED'  - forward/right/down (a missile's own nose/right/earthward
    %       axes when flying outbound): X = forward (unchanged), Y =
    %       right, Z = down.
    %   'FDR'  - forward/down/right (the munition's own tail-to-nose/
    %       earthward/right-side body axes): X = forward (unchanged), Y =
    %       down, Z = right. Used by DynamicsMPMM/AeroMPMM.
    %
    % 'NED' is related to 'Z-Up' by R = diag(1,-1,-1): a proper 180-degree
    % ROTATION about the X axis (det(R) = +1), not a mirror. That matters:
    % a rotation preserves every cross-product-based effect (Magnus force,
    % spin-damping torque, yaw-of-repose) automatically, because
    % R*(a x b) = (R*a) x (R*b) for any proper rotation. So a
    % Dynamics/Aero class only needs to (1) rotate its own fixed reference
    % vectors (gravity, "up") with gravityVector/upVector, and (2) rotate
    % an ANGLE that represents a rotation about the Y axis (pitch,
    % elevation) with rotateAngle when it first turns that angle into a
    % state vector - everything else (drag, lift, Magnus, spin damping) is
    % built from cross/dot products of already-consistent state vectors
    % and comes out correctly rotated with NO other changes, by that same
    % covariance property. R is self-inverse (R*R = I), so the same
    % function converts a vector either direction.
    %
    % 'FDR' is related to 'Z-Up' by M = [1 0 0; 0 0 -1; 0 -1 0]: a
    % REFLECTION (det(M) = -1), not a rotation - forward/down/right, taken
    % in that order, is left-handed (forward x down = left, not right;
    % see CoordinateFrame's own derivation notes / the conversation that
    % introduced this frame). M is still self-inverse (M*M = I), so
    % rotateVector/rotateAngle work the same both directions, but the
    % automatic cross-product covariance above does NOT hold for 'FDR':
    % M*(a x b) = -M(a) x M(b). Every genuine cross-product physics term
    % that operates on 'FDR'-native vectors (DynamicsMPMM's Coriolis,
    % AeroMPMM's yaw-of-repose and Magnus equations) has its sign flipped
    % to compensate - see those classes' own comments at each cross(...)
    % call for the specific correction applied.
    methods (Static)
        function v = rotateVector(v, frame)
            % Rotates one 3-vector (position or velocity) between 'Z-Up'
            % and 'NED'/'FDR'. Safe to call with frame omitted/'Z-Up' -
            % returns v unchanged.
            if nargin > 1 && strcmpi(frame, 'NED')
                v(2) = -v(2);
                v(3) = -v(3);
            elseif nargin > 1 && strcmpi(frame, 'FDR')
                tmp  = v(2);
                v(2) = -v(3);
                v(3) = -tmp;
            end
        end

        function angle = rotateAngle(angle, frame)
            % Rotates an angle that represents a rotation ABOUT THE Y AXIS
            % (pitch, elevation) between 'Z-Up' and 'NED'/'FDR' - Y flips
            % sign (NED) or swaps into Z with a sign flip (FDR), so the
            % angle needed to represent the same physical rotation flips
            % sign too in both cases. Does NOT apply to azimuth/spin
            % (rotations about Z/X in 'Z-Up'), which are unaffected. Note:
            % for 'FDR' this value is diagnostic-only (DynamicsMPMM.
            % buildLaunchState stores it in IDX_EULER, which is never read
            % back by computeDerivatives - see that class' comment), so
            % this mirrors 'NED''s sign convention rather than chasing a
            % fully rigorous Euler-angle re-derivation that nothing consumes.
            if nargin > 1 && (strcmpi(frame, 'NED') || strcmpi(frame, 'FDR'))
                angle = -angle;
            end
        end

        function g_vec = gravityVector(g, frame)
            % The gravity acceleration vector (pointing toward the
            % ground) in the given frame.
            g_vec = CoordinateFrame.rotateVector([0; 0; -g], frame);
        end

        function v = upVector(frame)
            % Unit vector pointing away from the ground (skyward) in the
            % given frame.
            v = CoordinateFrame.rotateVector([0; 0; 1], frame);
        end

        function v = lateralAxis(frame)
            % Fixed crossrange/lateral reference vector used to build a
            % planar lift direction (lift_dir = vel_dir x lateralAxis in
            % every Aero class) - a FIXED reference vector like gravity/
            % up, so it must be rotated with the frame same as those, not
            % hardcoded, for lift_dir to come out correctly rotated.
            v = CoordinateFrame.rotateVector([0; 1; 0], frame);
        end

        function h = altitude(X, frame)
            % Height above the ground from a state vector's position
            % components (X(1:3), or a bare 3-vector). 'FDR' keeps "down"
            % positive like 'NED', just at index 2 instead of 3 (see this
            % file's class comment) - callers must read the matching
            % column (X(2) worth) themselves; this just negates it.
            if nargin > 1 && strcmpi(frame, 'NED')
                h = -X(3);
            elseif nargin > 1 && strcmpi(frame, 'FDR')
                h = -X(2);
            else
                h = X(3);
            end
        end

        function alt = altitudeColumn(z, frame)
            % Vectorized form of altitude() for a whole time series, for
            % post-flight analysis like an apogee lookup - altitude()
            % itself takes one position/state vector at a time. Caller
            % passes the correct column for the frame: telemetry.pos(:,3)
            % for 'Z-Up'/'NED', telemetry.pos(:,2) for 'FDR' (see
            % altitude()) - this just negates it for 'NED'/'FDR'.
            if nargin > 1 && (strcmpi(frame, 'NED') || strcmpi(frame, 'FDR'))
                alt = -z;
            else
                alt = z;
            end
        end
    end
end
