%% File: Munition.m
classdef Munition < handle
    properties
        Name           (1,:) char = 'Munition'
        Type           (1,:) char = 'Unguided' % free-form role label, e.g. 'Guided', 'Airburst', 'TP', 'CourseCorrection', 'HE' - not a fixed enum
        Dynamics                  = []  % Holds instance of BaseDynamics subclass
        % A BaseExternalModel subclass (e.g. ExternalModelSimulink, or a
        % future .exe adapter) - an ALTERNATIVE to Dynamics, not a
        % complement to it: Dynamics.computeDerivatives is a per-RK4-
        % sub-step "what's the derivative right now" call Simulation
        % steps itself, whereas an external tool (Simulink's sim(), a
        % standalone .exe) runs its OWN internal time-stepping and hands
        % back a finished trajectory in one call. When this is set,
        % Simulation.run() bypasses its own step loop entirely and
        % delegates the whole flight to this instead - see
        % Simulation.runExternalModel. [] (default) = use Dynamics as
        % normal, exactly as before this property existed.
        ExternalModel             = []
        Aero                      = []  % Holds instance of BaseAero subclass
        Propulsion                = []  % Holds instance of BasePropulsion subclass
        Sensors        cell       = {}  % Cell array of BaseSensor subclasses
        Guidance                  = []  % Holds instance of BaseGuidance subclass
        Controller                = []  % Holds instance of BaseController subclass
        Charge                    = []  % Holds a ChargeZone (gun-fired munitions only; empty for rocket motors)
        MassProperties struct = struct('Mass', 3.0, 'Inertia', eye(3), ...
            'MassEmpty', 3.0, 'CGEmpty', [0;0;0], ...
            'CGFull', [0;0;0], 'CGRef', [0;0;0], ...
            'InertiaFull', eye(3), 'InertiaEmpty', eye(3))
        TwistRate      (1,1) double = 0.0 % rifling twist, calibers/turn; 0 = not applicable (fin-stabilized/no spin)
        LastGNCBus     struct     = struct()
        BurstTime                 = []  % s since launch at which fragmentation triggers; [] = never bursts
        SelfDestructTime          = []  % s since launch; hard ceiling on BurstTime - forces a burst at this time regardless of BurstTime (unset or programmed later), so the round can't fly on as an unexploded dud. [] = no ceiling. See Simulation.checkTermination and FireControlComputer.solveBurstTimeForStandoff.
        ParticleModel             = []  % BaseParticleModel subclass instance, consumed by spawnParticles
        FragmentationType (1,:) char = 'pre-fragmented';
    end

    methods
        function obj = Munition(name)
            if nargin > 0, obj.Name = name; end
        end

        function X0 = buildLaunchState(obj, launcher_pos, v0, elevation, azimuth, spin_init, inertiaMatrix)
            % Assembles the Dynamics-specific launch state, adding the
            % muzzle spin rate (from TwistRate + the round's bore
            % diameter) on top for a DynamicsMPMM or DynamicsMPMM
            % munition - so callers get one call site regardless of
            % whether the munition spins or not.
            if (isa(obj.Dynamics, 'DynamicsMPMM') || isa(obj.Dynamics, 'DynamicsMPMM')) && obj.TwistRate > 0
                spin0 = DynamicsMPMM.spinRateFromTwist(v0, obj.TwistRate, obj.Aero.RefLength);
                X0 = obj.Dynamics.buildLaunchState(launcher_pos, v0, elevation, azimuth, obj.MassProperties.Mass, spin0);
            elseif (isa(obj.Dynamics, 'Dynamics6DoF'))
                X0 = obj.Dynamics.buildLaunchState(launcher_pos, v0, elevation, azimuth, obj.MassProperties.Mass, spin_init, inertiaMatrix);
            else
                X0 = obj.Dynamics.buildLaunchState(launcher_pos, v0, elevation, azimuth, obj.MassProperties.Mass);
            end

            obj.applyFormFactors(elevation);
        end

        function applyFormFactors(obj, elevation)
            % STANAG 4355 IForm/FL (see AeroMPMM, MunitionData.formFactor)
            % are a polynomial in quadrant elevation - only evaluable once
            % QE is known, and buildLaunchState's own `elevation` argument
            % IS that QE (the caller already solved for it and converted
            % mils->radians before calling buildLaunchState - see
            % FireControlComputer's mil/rad boundary comment). Applying it
            % here means every runner gets this for free automatically,
            % instead of each one needing to repeat the manual "solve,
            % then MunitionData.formFactor, then set Aero.IForm/FL"
            % snippet runner_m107.m used to require by hand.
            %
            % A no-op whenever there's nothing to apply: no Aero, or no
            % data/<Name>.mat file for this munition (MunitionData.load
            % returns [] - the standard "optional override, fall back to
            % whatever's already configured" contract every MunitionData/
            % FactoryData accessor already has). IForm/FL are on EVERY
            % Aero class now (inherited from BaseAero) - AeroMPMM applies
            % them internally, every other class picks them up generically
            % via BaseAero.applyFormFactors in Munition.getDerivatives, so
            % there's no per-class isprop check needed here anymore. The
            % fallback `default` passed to formFactor is the CURRENT
            % IForm/FL value, not a hardcoded 1.0 - so a zone/field
            % genuinely missing from the data file leaves whatever was
            % already configured untouched, rather than silently
            % resetting it.
            if isempty(obj.Aero)
                return;
            end
            mdata = MunitionData.load(obj.Name);
            if isempty(mdata)
                return;
            end
            qe_mils = FireControlComputer.rad2mil(elevation);
            zone_name = '';
            if ~isempty(obj.Charge)
                zone_name = obj.Charge.Name;
            end
            obj.Aero.IForm = MunitionData.formFactor(mdata, 'iform', zone_name, qe_mils, obj.Aero.IForm);
            obj.Aero.FL    = MunitionData.formFactor(mdata, 'fl',    zone_name, qe_mils, obj.Aero.FL);
        end

        function updateGNC(obj, t, env, target)
            sensor_bus = struct();
            for i = 1:numel(obj.Sensors)
                s = obj.Sensors{i};
                sensor_bus.(s.Name) = s.step(t, obj.Dynamics.CurrentState, target, env);
            end

            nav_bus = obj.Dynamics.CurrentState;

            if ~isempty(obj.Guidance)
                guid_bus = obj.Guidance.step(t, nav_bus, target, sensor_bus);
            else
                guid_bus = struct('AccelCmd', 0.0);
            end

            if ~isempty(obj.Controller)
                ctrl_bus = obj.Controller.step(t, guid_bus, nav_bus, obj, env);
            else
                ctrl_bus = struct('FinDeflection', 0.0);
            end

            obj.LastGNCBus.Sensors    = sensor_bus;
            obj.LastGNCBus.Nav        = nav_bus;
            obj.LastGNCBus.Guidance   = guid_bus;
            obj.LastGNCBus.Controller = ctrl_bus;
        end

        function dX = getDerivatives(obj, t, X, env)
            obj.Dynamics.CurrentState = X;

            if ~isempty(obj.Propulsion)
                [F_prop, M_prop, m_dot] = obj.Propulsion.getThrust(t, env);
            else
                F_prop = [0;0;0]; M_prop = [0;0;0]; m_dot = 0;
            end

            if isfield(obj.LastGNCBus, 'Controller') && isfield(obj.LastGNCBus.Controller, 'FinDeflection')
                fin_deflection = obj.LastGNCBus.Controller.FinDeflection;
            else
                fin_deflection = 0.0;
            end

            if ~isempty(obj.Aero)
                [F_aero, M_aero] = obj.Aero.getForcesAndMoments(t, X, env, fin_deflection);
                if ~isa(obj.Aero, 'AeroMPMM') && ~isa(obj.Aero, 'AeroMissile6DoF')
                    F_aero = BaseAero.applyFormFactors(F_aero, X(4:6), obj.Aero.IForm, obj.Aero.FL);
                end
            else
                F_aero = [0;0;0]; M_aero = [0;0;0];
            end

            % --- DYNAMIC CG AND INERTIA CALCULATION ---
            m_current = X(BaseDynamics.IDX_MASS);

            if isfield(obj.MassProperties, 'MassEmpty') && isfield(obj.MassProperties, 'CGEmpty') && ...
                    isfield(obj.MassProperties, 'CGFull') && isfield(obj.MassProperties, 'CGRef')

                m_empty  = obj.MassProperties.MassEmpty;
                m_full   = obj.MassProperties.Mass;
                cg_empty = obj.MassProperties.CGEmpty;
                cg_full  = obj.MassProperties.CGFull;
                cg_ref   = obj.MassProperties.CGRef;

                if (m_full - m_empty) > 1e-4
                    mass_frac = (m_current - m_empty) / (m_full - m_empty);
                    cg_prop = (m_full * cg_full - m_empty * cg_empty) / (m_full - m_empty);
                    cg_current = (m_empty * cg_empty + (m_current - m_empty) * cg_prop) / m_current;

                    if isfield(obj.MassProperties, 'InertiaFull') && isfield(obj.MassProperties, 'InertiaEmpty')
                        obj.MassProperties.Inertia = obj.MassProperties.InertiaEmpty + ...
                            mass_frac * (obj.MassProperties.InertiaFull - obj.MassProperties.InertiaEmpty);
                    end
                else
                    cg_current = cg_empty;
                    if isfield(obj.MassProperties, 'InertiaEmpty')
                        obj.MassProperties.Inertia = obj.MassProperties.InertiaEmpty;
                    end
                end

                % 3. Calculate distance vector FROM the new CG TO the reference point
                cg_shift = cg_ref - cg_current;

                % 4. Shift Moments via cross product (M_true = M_ref + r x F)
                M_aero = M_aero + cross(cg_shift, F_aero);
                M_prop = M_prop + cross(cg_shift, F_prop);
            end

            forces.Propulsion    = F_prop;
            forces.Aero          = F_aero;
            forces.FinDeflection = fin_deflection;
            moments.Propulsion   = M_prop;
            moments.Aero         = M_aero;
            dX = obj.Dynamics.computeDerivatives(t, X, forces, moments, obj.MassProperties, m_dot);
        end

        function particles = spawnParticles(obj, X, ~)
            % Releases obj.ParticleModel's particles at burst state X.
            % Builds a body frame from the velocity vector: forward/spin
            % axis = velocity unit vector, same coincident-spin-axis
            % assumption DynamicsMPMM already documents, so this adds
            % no new approximation. The lateral axes are an arbitrary (but
            % consistent) Gram-Schmidt completion - the particle pattern is
            % rotationally symmetric about this axis, so the true (untracked)
            % roll angle at burst has no physical effect on the result.
            %
            % Each particle's velocity is rigid-body kinematics
            % v = v_cg + spin_rate*(fwd x r) plus the model's own
            % ejectionVelocities() (zero for ForwardExplosionParticleModel).
            if isempty(obj.ParticleModel)
                error('Munition:spawnParticles', '%s has no ParticleModel set.', obj.Name);
            end

            pos = X(1:3);
            vel = X(4:6);
            v_mag = norm(vel);
            if v_mag > 1e-3
                fwd = vel / v_mag;
            else
                fwd = [1; 0; 0];
            end
            if numel(X) >= 9
                spin_rate = X(9);
            else
                spin_rate = 0.0;
            end

            % A Dynamics class with no Frame property at all (e.g.
            % DynamicsMPMM) is always-FDR by its own design (see that
            % class' comment), not Z-Up - same fallback FireControlComputer
            % uses for the same reason (see its own comment there).
            if isprop(obj.Dynamics, 'Frame')
                frame = obj.Dynamics.Frame;
            elseif obj.Dynamics.StateDimension == BaseDynamics.FULL_STATE_DIM
                frame = 'FDR';
            else
                frame = 'Z-Up';
            end

            up_ref = CoordinateFrame.upVector(frame);
            if abs(dot(fwd, up_ref)) > 0.99
                up_ref = [1; 0; 0];
            end
            lat1 = cross(up_ref, fwd); lat1 = lat1 / norm(lat1);
            lat2 = cross(fwd, lat1);
            R = [fwd, lat1, lat2]; % body -> world, columns are body axes in world coords

            model = obj.ParticleModel;
            n = model.numParticles();
            v_ejection_body = model.ejectionVelocities();

            particles = cell(n, 1);
            for i = 1:n
                r_world = R * model.Positions(i, :)';
                v_kick_world = R * v_ejection_body(i, :)';
                v_spin_world = cross(spin_rate * fwd, r_world);

                p_pos = pos + r_world;
                p_vel = vel + v_spin_world + v_kick_world;

                p = Particle(sprintf('%s_Frag%03d', obj.Name, i), ...
                    model.Shape, model.Dimensions, model.CD, model.ReferenceAreaMode, model.ParticleMass);
                if isprop(p.Dynamics, 'Frame'), p.Dynamics.Frame = frame; end
                if isprop(p.Aero, 'Frame'), p.Aero.Frame = frame; end

                pitch0 = 0.0;
                if norm(p_vel) > 1e-3
                    pitch0 = atan2(p_vel(3), p_vel(1));
                end
                p.Dynamics.InitialState = [p_pos; p_vel; pitch0; model.ParticleMass];

                particles{i} = p;
            end
        end
    end
end