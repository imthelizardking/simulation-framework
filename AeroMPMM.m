classdef AeroMPMM < BaseAero
    % High-fidelity STANAG 4355-style Modified Point-Mass Model aero: a
    % full yaw-of-repose VECTOR (alpha_e) treatment, rather than a
    % simplified planar/scalar-alpha buildup. Ported from a reference
    % calc_accs.m implementation (see example_code.m) - drag/lift/Magnus forces and
    % the spin-damping moment, all as functions of alpha_e rather than a
    % single in-plane angle of attack. Paired with DynamicsMPMM, which
    % supplies the real-gravity vector alpha_e needs (see
    % DynamicsMPMM.computeGravityVector) and integrates the resulting
    % forces/moment plus its own Coriolis term (Coriolis does NOT feed
    % back into alpha_e - see example_code.m, acc_coriolis is added to
    % the total acceleration directly, never through CMA).
    %
    % getForcesAndMoments below is deliberately just a short, ordered
    % list of calls into named methods (evaluateCoefficients,
    % applyBaseBleedShift, yawOfReposeEquation, dragEquation,
    % liftEquation, magnusEquation, spinDampingEquation) - each one is
    % the actual equation, on its own, so you can jump straight to it
    % (go-to-definition) instead of hunting through one large inline
    % block. Every coefficient is looked up ONCE, in ONE place
    % (evaluateCoefficients), rather than scattered across the force
    % equations that use them.
    %
    % COORDINATE FRAME NOTE: example_code.m's reference (STANAG 4355
    % Modified Point Mass Model, Turkish-commented) works in an
    % X-forward/Y-UP/Z-right frame (Earth's center sits at pos-[0;-R;0],
    % i.e. "down" is -Y there). This framework's Dynamics uses FDR
    % (X-forward, Y-DOWN, Z-right - see CoordinateFrame.m and
    % BaseDynamics' IDX_POS comment). Unlike the framework's other
    % (NED) axis convention, FDR is left-handed (forward x down = left,
    % not right - see CoordinateFrame.m's class comment), so every
    % cross-product-based formula below (alpha_e, Magnus) carries an
    % extra sign flip relative to a straight port of either the
    % reference's or NED's formulas - see the comment at each cross(...)
    % call below. The INPUT vectors (gravity, Coriolis) are computed
    % directly in FDR, not copied from the reference's Y-up formulas -
    % see DynamicsMPMM.computeGravityVector/computeCoriolisVector.
    %
    % NOTE: this class always works in FDR internally and ignores the
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
        % Polynomial/PiecewisePolynomial struct (see BaseAero.evalCoeff).
        % Evaluated at alpha_e's MAGNITUDE (a genuine 3D vector angle), not an in-
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
        % below, which handles the drag-multiplier effect; this block instead shifts
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

        % Optional CD0 multiplier while a base-bleed unit burns - via the
        % Propulsion_DYU contract (obj.BaseBleed.dragMultiplier(t)). Independent of
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
            flow = BaseAero.computeFlowState(X, env, 'FDR');
            if flow.v_mag < 1e-3
                F_aero = [0; 0; 0];
                M_aero = [0; 0; 0];
                return;
            end
            va_b = flow.vel;
            Va   = flow.v_mag;
            mach = flow.mach;
            ro    = env.getDensity(CoordinateFrame.altitude(X, 'FDR'));
            p     = X(BaseDynamics.IDX_BODYRATE(1));
            m_now = X(BaseDynamics.IDX_MASS);

            coeffs = obj.evaluateCoefficients(mach);
            [coeffs, Ix_now] = obj.applyBaseBleedShift(coeffs, m_now);

            alpha_e = obj.yawOfReposeEquation(X, va_b, p, Ix_now, coeffs, ro, Va);
            alpha_e_mag = norm(alpha_e);
            obj.LastAlphaEMag = alpha_e_mag;

            F_drag   = obj.dragEquation(coeffs, alpha_e_mag, t, ro, Va, va_b);
            F_lift   = obj.liftEquation(coeffs, alpha_e_mag, alpha_e, ro, Va);
            F_magnus = obj.magnusEquation(coeffs, alpha_e, va_b, ro, p);
            F_aero   = F_drag + F_lift + F_magnus;

            M_roll = obj.spinDampingEquation(coeffs, ro, p, Va);
            M_aero = [M_roll; 0; 0];
        end

        function coeffs = evaluateCoefficients(obj, mach)
            % Every aero coefficient, evaluated at this Mach number, in
            % ONE place - the only spot in this class that calls
            % BaseAero.evalCoeff. Every equation below reads its
            % coefficients from the returned struct rather than
            % re-evaluating them itself.
            coeffs.CD0   = BaseAero.evalCoeff(obj.CD0,   mach);
            coeffs.CDA2  = BaseAero.evalCoeff(obj.CDA2,  mach);
            coeffs.CDA4  = BaseAero.evalCoeff(obj.CDA4,  mach);
            coeffs.CLA   = BaseAero.evalCoeff(obj.CLA,   mach);
            coeffs.CLA3  = BaseAero.evalCoeff(obj.CLA3,  mach);
            coeffs.CLA5  = BaseAero.evalCoeff(obj.CLA5,  mach);
            coeffs.CMA   = BaseAero.evalCoeff(obj.CMA,   mach);
            coeffs.CMA3  = BaseAero.evalCoeff(obj.CMA3,  mach);
            coeffs.CSPIN = BaseAero.evalCoeff(obj.CSPIN, mach);
            coeffs.CMAG  = BaseAero.evalCoeff(obj.CMAG,  mach);
        end

        function [coeffs, Ix_now] = applyBaseBleedShift(obj, coeffs, m_now)
            % Base-bleed CG/inertia shift (see BaseBleedOn property doc
            % above) - linear interpolation between launch and burnout
            % values by remaining fuel fraction, same shape as
            % example_code.m. A no-op (coeffs/obj.Ix unchanged) unless
            % BaseBleedOn is set.
            Ix_now = obj.Ix;
            if ~obj.BaseBleedOn || obj.InitialMass <= 0
                return;
            end
            M_burnout = obj.InitialMass - obj.BurnoutFuelMass;
            cg_f0  = obj.BurnoutCg + (obj.InitialCg - obj.BurnoutCg) * obj.InitialMass / (obj.InitialMass - M_burnout);
            cg_now = obj.BurnoutCg + (cg_f0 - obj.BurnoutCg) * (m_now - M_burnout) / m_now;
            Ix_now = obj.InitialInertia + (obj.InitialInertia - obj.BurnoutInertia) * ...
                (m_now - obj.InitialMass) / (obj.InitialMass - M_burnout);
            % CG offset shifts the overturning-moment coefficients - see
            % example_code.m's CMA/CMA3 correction.
            coeffs.CMA  = coeffs.CMA  + (cg_now - obj.InitialCg) * (coeffs.CD0 - coeffs.CLA) / obj.RefLength;
            coeffs.CMA3 = coeffs.CMA3 + (cg_now - obj.InitialCg) * (coeffs.CLA3 + coeffs.CDA2 - 0.5*coeffs.CLA) / obj.RefLength;
        end

        function alpha_e = yawOfReposeEquation(obj, X, va_b, p, Ix_now, coeffs, ro, Va)
            % "Bradley's formula" (STANAG 4355): the quasi-steady angle
            % between the nose axis and velocity that balances
            % gyroscopic precession against the aerodynamic overturning
            % moment. CMA_total uses the LAGGED |alpha_e| (LastAlphaEMag)
            % for its CMA3 term - see that property's comment for why
            % (avoids a circular solve within this one call).
            CMA_total = coeffs.CMA + coeffs.CMA3 * obj.LastAlphaEMag^2;
            acc_gravity = DynamicsMPMM.computeGravityVector(X(BaseDynamics.IDX_POS), obj.LatitudeDeg);
            denom = ro * obj.RefArea * obj.RefLength * CMA_total * Va^4;
            if abs(denom) < 1e-9 || abs(Ix_now) < 1e-12
                alpha_e = [0; 0; 0];
                return;
            end
            % FDR is left-handed (see this file's COORDINATE FRAME NOTE),
            % so this cross product carries an extra sign flip relative
            % to the formula's NED/reference form to represent the same
            % physical alpha_e vector.
            alpha_e = -(2 * Ix_now * p * cross(acc_gravity, va_b)) / denom;
        end

        function F_drag = dragEquation(obj, coeffs, alpha_e_mag, t, ro, Va, va_b)
            cd0_multiplier = 1.0;
            if ~isempty(obj.BaseBleed)
                cd0_multiplier = obj.BaseBleed.dragMultiplier(t);
            end
            CD_total = max((coeffs.CD0 * cd0_multiplier) ...
                + coeffs.CDA2 * (obj.QD * alpha_e_mag)^2 ...
                + coeffs.CDA4 * (obj.QD * alpha_e_mag)^4, 0);
            F_drag = -(obj.RefArea * ro * obj.IForm / 2) * CD_total * Va * va_b;
        end

        function F_lift = liftEquation(obj, coeffs, alpha_e_mag, alpha_e, ro, Va)
            CLA_total = coeffs.CLA + coeffs.CLA3 * alpha_e_mag^2 + coeffs.CLA5 * alpha_e_mag^4;
            F_lift = (obj.RefArea * ro * obj.FL / 2) * CLA_total * Va^2 * alpha_e;
        end

        function F_magnus = magnusEquation(obj, coeffs, alpha_e, va_b, ro, p)
            % FDR is left-handed (see this file's COORDINATE FRAME NOTE),
            % so this cross product's sign is flipped relative to the
            % formula's NED/reference form (dropping the leading minus)
            % to represent the same physical Magnus force.
            F_magnus = (obj.RefArea * ro * obj.RefLength * obj.QM * p * coeffs.CMAG) * cross(alpha_e, va_b) / 2;
        end

        function M_roll = spinDampingEquation(obj, coeffs, ro, p, Va)
            % Expressed so DynamicsMPMM's existing dspin = M_aero(1)/Ix
            % reproduces example_code.m's own spin_acc =
            % (pi*ro*D^4*p*Va*CSPIN)/(8*Ix) exactly.
            M_roll = (pi * ro * obj.RefLength^4 * p * Va * coeffs.CSPIN) / 8;
        end
    end
end
