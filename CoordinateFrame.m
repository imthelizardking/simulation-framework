%% File: CoordinateFrame.m
classdef CoordinateFrame
    % Shared conversion between the framework's two supported position/
    % velocity conventions:
    %   'Z-Up' (default) - the framework's original convention, unchanged.
    %       X = downrange, Y = crossrange, Z = up.
    %   'NED'  - forward/right/down (a missile's own nose/right/earthward
    %       axes when flying outbound): X = forward (unchanged), Y =
    %       right, Z = down.
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
    methods (Static)
        function v = rotateVector(v, frame)
            % Rotates one 3-vector (position or velocity) between 'Z-Up'
            % and 'NED'. Safe to call with frame omitted/'Z-Up' - returns
            % v unchanged.
            if nargin > 1 && strcmpi(frame, 'NED')
                v(2) = -v(2);
                v(3) = -v(3);
            end
        end

        function angle = rotateAngle(angle, frame)
            % Rotates an angle that represents a rotation ABOUT THE Y AXIS
            % (pitch, elevation) between 'Z-Up' and 'NED' - Y flips sign
            % under R, so the angle needed to represent the same physical
            % rotation flips sign too. Does NOT apply to azimuth/spin
            % (rotations about Z/X), which are unaffected by R.
            if nargin > 1 && strcmpi(frame, 'NED')
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
            % components (X(1:3), or a bare 3-vector).
            if nargin > 1 && strcmpi(frame, 'NED')
                h = -X(3);
            else
                h = X(3);
            end
        end

        function alt = altitudeColumn(z, frame)
            % Vectorized form of altitude() for a whole time series (e.g.
            % telemetry.pos(:,3)), for post-flight analysis like an
            % apogee lookup - altitude() itself takes one position/state
            % vector at a time.
            if nargin > 1 && strcmpi(frame, 'NED')
                alt = -z;
            else
                alt = z;
            end
        end
    end
end
