classdef AeroMPMM < BaseAero
    % High-fidelity STANAG 4355-style Modified Point-Mass Model aero: a
    % full yaw-of-repose VECTOR (alpha_e) treatment, instead of the
    % simplified planar/scalar-alpha buildup AeroTabulated/
    % AeroTabulatedSpin use. Ported from a reference calc_accs.m
    % implementation (see example_code.m) - drag/lift/Magnus forces and
    % the spin-damping moment, all as functions of alpha_e rather than a
    % single in-plane angle of attack. Paired with DynamicsMPMM, which
    % supplies the real-gravity vector alpha_e needs (see
    % DynamicsMPMM.computeGravityVector) and integrates the resulting
    % forces/moment plus its own Coriolis term (Coriolis does NOT feed
    % back into alpha_e - see example_code.m, acc_coriolis is added to
    % the total acceleration directly, never through CMA).
    %
    % COORDINATE FRAME NOTE: example_code.m's reference (STANAG 4355
    % Modified Point Mass Model, Turkish-commented) works in an
    % X-forward/Y-UP/Z-right frame (Earth's center sits at pos-[0;-R;0],
    % i.e. "down" is -Y there). This framework's Dynamics uses NED
    % (X-forward, Y-right, Z-DOWN - see CoordinateFrame.m and
    % BaseDynamics' IDX_POS comment). The two are related by a consistent
    % axis relabeling (X unchanged, Y_ned=Z_ref, Z_ned=-Y_ref) that
    % preserves right-handedness, so every cross-product-based formula
    % below (alpha_e, Magnus) carries over unchanged once the INPUT
    % vectors (gravity, Coriolis) are computed directly in NED rather
    % than copying the reference's Y-up formulas verbatim - see
    % DynamicsMPMM.computeGravityVector/computeCoriolisVector for the
    % re-derived NED versions this class and DynamicsMPMM both call.
    %
    % NOTE: this class always works in NED internally and ignores the
    % inherited Frame property from BaseAero (which still defaults to
    % 'Z-Up' but has no effect here) - matches DynamicsMPMM, which has no
    % Frame property at all for the same reason (see that class' comment).
    properties
        % Yaw-of-repose axial inertia (kg*m^2). getForcesAndMoments has
        % no access to mass_prop (unlike Dynamics.computeDerivatives), so
        % this is kept in sync with the paired Munition's
        % MassProperties.Inertia(1,1) manually - same "must match the
        % paired class" contract CoordinateFrame's Frame property already
        % has elsewhere in this framework. Ignored (recomputed
        % internally) while BaseBleedOn drifts inertia over the flight -
        % see BaseBleedOn below.
        Ix (1,1) double = 1.0

        % Launch latitude (degrees) - must match DynamicsMPMM's
        % InitLatitudeDeg on the paired Dynamics object. Feeds the real-
        % gravity term alpha_e needs (see DynamicsMPMM.computeGravityVector).
        LatitudeDeg (1,1) double = 0.0

        % Aero coefficients - scalar, or a Mach-dependent Table/
        % Polynomial/PiecewisePolynomial struct (see BaseAero.evalCoeff),
        % same representation AeroTabulatedSpin uses. Evaluated at
        % alpha_e's MAGNITUDE (a genuine 3D vector angle), not an in-
        % plane scalar alpha. CD0 itself is inherited from BaseAero (zero-
        % yaw drag) - not redeclared here.
        CDA2  = 0      % drag rise with (QD*|alpha_e|)^2
        CDA4  = 0      % drag rise with (QD*|alpha_e|)^4
        CLA   = 2.0    % normal-force slope
        CLA3  = 0      % normal-force cubic term (|alpha_e|^2 factor on CLA)
        CLA5  = 0      % normal-force quintic term (|alpha_e|^4 factor on CLA)
        CMA   = 2.0    % overturning-moment slope - drives alpha_e itself
        % Overturning-moment cubic term. Feeding alpha_e's own magnitude
        % back into the coefficient that SOLVES for alpha_e would be
        % circular within one call, so - matching example_code.m's own
        % "method 1: direct approach, no fixed-point iter" - CMA3 is
        % evaluated at the PREVIOUS call's |alpha_e| (LastAlphaEMag
        % below), not the value being solved for this call. CLA3/CLA5
        % don't need this: they're only used AFTER alpha_e is solved, on
        % the fresh value, so no lag there.
        CMA3  = 0
        CSPIN = -0.03  % spin-damping coefficient
        CMAG  = 1.0    % Magnus force coefficient

        % Fitting/form factors (STANAG 4355 "iform"/"fL"/"QD"/"QM") - unit
        % gain (1.0) reproduces the textbook formulas below exactly;
        % override to tune against real firing-table data.
        IForm = 1.0    % drag form factor
        FL    = 1.0    % lift form factor
        QD    = 1.0    % drag-buildup alpha_e scaling
        QM    = 1.0    % Magnus scaling

        % Base-bleed CG/inertia shift (STANAG 4355's own optional
        % refinement - distinct from the CD0-reduction BaseBleed property
        % on AeroTabulated/AeroTabulatedSpin, which you can ALSO attach
        % here for the drag-multiplier effect; this block instead shifts
        % the overturning-moment coefficients and the yaw-of-repose Ix as
        % the bleed unit burns and the CG moves). Off by default -
        % nothing below is read unless BaseBleedOn is true.
        BaseBleedOn      (1,1) logical = false
        InitialMass      (1,1) double = 0   % launch mass (kg), before any bleed burn
        InitialCg        (1,1) double = 0   % launch CG station (m), same reference as RefLength
        InitialInertia   (1,1) double = 0   % launch (pre-burn) axial inertia (kg*m^2)
        BurnoutFuelMass  (1,1) double = 0   % bleed-unit fuel mass consumed by burnout (kg)
        BurnoutCg        (1,1) double = 0   % CG station at burnout (m)
        BurnoutInertia   (1,1) double = 0   % axial inertia at burnout (kg*m^2)

        % Optional CD0 multiplier while a base-bleed unit burns - same
        % Propulsion_DYU contract AeroTabulated/AeroTabulatedSpin already
        % use (obj.BaseBleed.dragMultiplier(t)). Independent of
        % BaseBleedOn above: that flag is only the CG/inertia-shift
        % refinement, this is only the CD0 reduction.
        BaseBleed = []
    end

    properties (Access = private)
        % |alpha_e| from the previous getForcesAndMoments call - see the
        % CMA3 property comment above.
        LastAlphaEMag (1,1) double = 0
    end

    methods
        function [F_aero, M_aero] = getForcesAndMoments(obj, t, X, env, ~)
            flow = BaseAero.computeFlowState(X, env, 'NED');
            if flow.v_mag < 1e-3
                F_aero = [0; 0; 0];
                M_aero = [0; 0; 0];
                return;
            end
            va_b = flow.vel;
            Va   = flow.v_mag;
            mach = flow.mach;
            ro   = env.getDensity(CoordinateFrame.altitude(X, 'NED'));
            p    = X(BaseDynamics.IDX_BODYRATE(1));
            m_now = X(BaseDynamics.IDX_MASS);

            CD0_v  = BaseAero.evalCoeff(obj.CD0,  mach);
            CDA2_v = BaseAero.evalCoeff(obj.CDA2, mach);
            CDA4_v = BaseAero.evalCoeff(obj.CDA4, mach);
            CLA_v  = BaseAero.evalCoeff(obj.CLA,  mach);
            CLA3_v = BaseAero.evalCoeff(obj.CLA3, mach);
            CLA5_v = BaseAero.evalCoeff(obj.CLA5, mach);
            CMA_v  = BaseAero.evalCoeff(obj.CMA,  mach);
            CMA3_v = BaseAero.evalCoeff(obj.CMA3, mach);
            CSPIN_v = BaseAero.evalCoeff(obj.CSPIN, mach);
            CMAG_v  = BaseAero.evalCoeff(obj.CMAG,  mach);

            % Base-bleed CG/inertia shift (see BaseBleedOn doc above) -
            % linear interpolation between launch and burnout values by
            % remaining fuel fraction, same shape as example_code.m.
            Ix_now = obj.Ix;
            if obj.BaseBleedOn && obj.InitialMass > 0
                M_burnout = obj.InitialMass - obj.BurnoutFuelMass;
                cg_f0  = obj.BurnoutCg + (obj.InitialCg - obj.BurnoutCg) * obj.InitialMass / (obj.InitialMass - M_burnout);
                cg_now = obj.BurnoutCg + (cg_f0 - obj.BurnoutCg) * (m_now - M_burnout) / m_now;
                Ix_now = obj.InitialInertia + (obj.InitialInertia - obj.BurnoutInertia) * ...
                    (m_now - obj.InitialMass) / (obj.InitialMass - M_burnout);
                % CG offset shifts the overturning-moment coefficients -
                % see example_code.m's CMA/CMA3 correction.
                CMA_v  = CMA_v  + (cg_now - obj.InitialCg) * (CD0_v - CLA_v) / obj.RefLength;
                CMA3_v = CMA3_v + (cg_now - obj.InitialCg) * (CLA3_v + CDA2_v - 0.5*CLA_v) / obj.RefLength;
            end

            % Yaw of repose ("Bradley's formula", STANAG 4355): the
            % quasi-steady angle between the nose axis and velocity that
            % balances gyroscopic precession against the aerodynamic
            % overturning moment. CMA_total uses the LAGGED |alpha_e|
            % (LastAlphaEMag) for its CMA3 term - see the CMA3 property
            % comment for why.
            CMA_total = CMA_v + CMA3_v * obj.LastAlphaEMag^2;
            acc_gravity = DynamicsMPMM.computeGravityVector(X(BaseDynamics.IDX_POS), obj.LatitudeDeg);
            denom = ro * obj.RefArea * obj.RefLength * CMA_total * Va^4;
            if abs(denom) < 1e-9 || abs(Ix_now) < 1e-12
                alpha_e = [0; 0; 0];
            else
                alpha_e = (2 * Ix_now * p * cross(acc_gravity, va_b)) / denom;
            end
            alpha_e_mag = norm(alpha_e);
            obj.LastAlphaEMag = alpha_e_mag;

            cd0_multiplier = 1.0;
            if ~isempty(obj.BaseBleed)
                cd0_multiplier = obj.BaseBleed.dragMultiplier(t);
            end
            CD_total  = max((CD0_v * cd0_multiplier) + CDA2_v*(obj.QD*alpha_e_mag)^2 + CDA4_v*(obj.QD*alpha_e_mag)^4, 0);
            CLA_total = CLA_v + CLA3_v*alpha_e_mag^2 + CLA5_v*alpha_e_mag^4;

            F_drag   = -(obj.RefArea*ro*obj.IForm/2) * CD_total * Va * va_b;
            F_lift   =  (obj.RefArea*ro*obj.FL/2) * CLA_total * Va^2 * alpha_e;
            F_magnus = -(obj.RefArea*ro*obj.RefLength*obj.QM*p*CMAG_v) * cross(alpha_e, va_b) / 2;
            F_aero = F_drag + F_lift + F_magnus;

            % Spin-damping moment, expressed so DynamicsMPMM's existing
            % dspin = M_aero(1)/Ix reproduces example_code.m's own
            % spin_acc = (pi*ro*D^4*p*Va*CSPIN)/(8*Ix) exactly.
            M_roll = (pi * ro * obj.RefLength^4 * p * Va * CSPIN_v) / 8;
            M_aero = [M_roll; 0; 0];
        end
    end
end
