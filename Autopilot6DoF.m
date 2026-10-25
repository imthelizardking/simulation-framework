classdef Autopilot6DoF < BaseController
    % Multi-axis (pitch+yaw) autopilot for AeroMissile6DoF/Dynamics6DoF:
    % converts guidance's INERTIAL acceleration command into pitch/yaw fin
    % deflections. Unlike Controller_LongitudinalFin (which treats fin
    % deflection as an idealized DIRECT addition to angle of attack -
    % adequate for Dynamics3Dof's pitch-lag model), this class' aero
    % (AeroMissile6DoF) gives fins genuine MOMENT authority only
    % (Cm_delta/Cn_delta) - a fin rotates the body, which changes alpha/
    % beta, which changes the aerodynamic force. That's a real (if still
    % simplified) control chain, not a one-step lookup, so this is a
    % two-loop design: outer loop converts the desired lateral
    % acceleration into a desired alpha/beta (inverting CN_alpha/CY_beta,
    % same "required angle to produce this force" idea
    % Controller_LongitudinalFin already uses); inner loop is a
    % proportional-derivative law on angle-of-attack error and body rate,
    % converted to a required moment coefficient, then solved for the fin
    % deflection that produces it via Cm_delta/Cn_delta.
    %
    % The rate-derivative (Kd_rate) term doubles as compensation for
    % AeroMissile6DoF's own Cm_q/Cn_r defaulting to 0 (no real DATCOM
    % $DAMP data yet - see that class) - providing damping through active
    % control feedback is a real, common autopilot architecture choice,
    % not just a workaround, but Kp_alpha/Kd_rate below are rough,
    % UNTUNED starting gains, not derived from any stability margin
    % analysis. Roll is left uncontrolled (Cl_delta unused) - for a true
    % axisymmetric cruciform airframe, roll ORIENTATION doesn't affect
    % aero performance, and the accel-command projection below uses the
    % vehicle's actual current DCM every step, so it stays correctly
    % resolved into body pitch/yaw axes regardless of whatever roll angle
    % the vehicle happens to drift to.
    properties
        % Sized so the FULL MaxAlphaCmd error maps to roughly
        % MaxDeflection (0.349 rad) given the illustrative Cm_delta=-0.8
        % placeholder (Kp*MaxAlphaCmd/|Cm_delta| ~ MaxDeflection) - with
        % the old default of 4.0, any AoA error past ~4 degrees already
        % saturated the fin, making the autopilot bang-bang almost
        % continuously rather than tracking smoothly. Re-tune once real
        % Cm_delta/Cn_delta values replace the placeholder - this is
        % matched to THAT number, not a universal constant.
        Kp_alpha (1,1) double = 1.0  % proportional gain, alpha/beta error -> commanded moment coefficient
        Kd_rate  (1,1) double = 0.3  % rate-damping gain (see class comment on why this substitutes for native Cm_q/Cn_r)
        % Angle-of-attack/sideslip limiter - inverting CN_alpha/CY_beta to
        % find the "angle that would produce this force" blows up at low
        % dynamic pressure (e.g. right after a slow rail launch): a small,
        % perfectly achievable accel command can invert to hundreds of
        % degrees, since q_bar is tiny and the linear force model has no
        % upper bound built in. Without this clamp the fins saturate at
        % MaxDeflection continuously trying to chase an unreachable
        % target angle - verified numerically to actually happen at this
        % class' own default launch speed. Every real missile autopilot
        % limits commanded AoA for the same reason (both to stay in the
        % linear aero regime and because low q_bar genuinely can't
        % produce large forces without an absurd angle).
        MaxAlphaCmd (1,1) double = deg2rad(15)
    end

    methods
        function ctrl_bus = step(obj, ~, guid_bus, nav_state, veh_cfg, env)
            q = nav_state(BaseDynamics.IDX_BODYRATE(2));
            r = nav_state(BaseDynamics.IDX_BODYRATE(3));
            vel = nav_state(BaseDynamics.IDX_VEL);
            m   = nav_state(BaseDynamics.IDX_MASS);

            % Quaternion (Dynamics6DoF.IDX_QUAT), not the frozen IDX_EULER
            % launch-time snapshot - see that class' header comment.
            R_bi = Dynamics6DoF.quatToDCM(nav_state(Dynamics6DoF.IDX_QUAT));
            v_body = R_bi.' * vel;
            V = norm(v_body);
            if V < 1e-3
                ctrl_bus = struct('FinDeflection', struct('Pitch', 0.0, 'Yaw', 0.0, 'Roll', 0.0));
                return;
            end
            alpha = atan2(v_body(3), v_body(1));
            beta  = asin(max(-1, min(1, v_body(2) / V)));

            % Project the INERTIAL accel command into the vehicle's
            % CURRENT body axes (not an assumed-zero-roll approximation -
            % see class comment) so the pitch/yaw split stays correct
            % regardless of roll angle.
            a_cmd_body = R_bi.' * guid_bus.AccelCmd_Inertial;

            altitude = CoordinateFrame.altitude(nav_state, 'NED');
            ro    = env.getDensity(altitude);
            q_bar = 0.5 * ro * V^2;
            aero  = veh_cfg.Aero;

            % Desired alpha/beta from desired body-frame lateral accel -
            % inverts AeroMissile6DoF's own force convention
            % (F_body = q_bar*RefArea*[-CA; CY; -CN]) so the sign here
            % matches that class exactly.
            denom_pitch = q_bar * aero.RefArea * aero.CN_alpha;
            denom_yaw   = q_bar * aero.RefArea * aero.CY_beta;
            if abs(denom_pitch) > 1e-6
                alpha_cmd = -a_cmd_body(3) * m / denom_pitch;
            else
                alpha_cmd = 0.0;
            end
            if abs(denom_yaw) > 1e-6
                beta_cmd = a_cmd_body(2) * m / denom_yaw;
            else
                beta_cmd = 0.0;
            end
            alpha_cmd = max(min(alpha_cmd, obj.MaxAlphaCmd), -obj.MaxAlphaCmd);
            beta_cmd  = max(min(beta_cmd,  obj.MaxAlphaCmd), -obj.MaxAlphaCmd);

            Cm_cmd = obj.Kp_alpha * (alpha_cmd - alpha) - obj.Kd_rate * q;
            Cn_cmd = obj.Kp_alpha * (beta_cmd - beta)   - obj.Kd_rate * r;

            if abs(aero.Cm_delta) > 1e-9
                d_pitch = Cm_cmd / aero.Cm_delta;
            else
                d_pitch = 0.0; % no fin authority configured - see AeroMissile6DoF.Cm_delta
            end
            if abs(aero.Cn_delta) > 1e-9
                d_yaw = Cn_cmd / aero.Cn_delta;
            else
                d_yaw = 0.0;
            end

            d_pitch = max(min(d_pitch, obj.MaxDeflection), -obj.MaxDeflection);
            d_yaw   = max(min(d_yaw,   obj.MaxDeflection), -obj.MaxDeflection);

            ctrl_bus = struct('FinDeflection', struct('Pitch', d_pitch, 'Yaw', d_yaw, 'Roll', 0.0));
        end
    end
end
