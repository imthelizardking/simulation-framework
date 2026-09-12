%% File: FireControlComputer.m
classdef FireControlComputer < handle
    properties
        Gravity      (1,1) double = 9.80665
        ElevationMin (1,1) double = deg2rad(5.0)   % Launcher physical lower limit
        ElevationMax (1,1) double = deg2rad(85.0)  % Launcher physical upper limit
        AzimuthMin   (1,1) double = -pi             % Traverse limit; -pi/pi = unrestricted (turret/AA mount)
        AzimuthMax   (1,1) double = pi               % Narrower for a fixed-arc mount (e.g. towed carriage)
        CoordFrame   (1,:) char   = 'Z-Up'         % 'Z-Up' or 'NED'
        
        % Numerical Shooting Parameters
        MaxIterations (1,1) double = 12
        ToleranceMeters (1,1) double = 0.5         % Stop when miss distance < 0.5 m
    end
    
    methods
        function obj = FireControlComputer(gravity, coord_frame)
            if nargin > 0, obj.Gravity = gravity; end
            if nargin > 1, obj.CoordFrame = coord_frame; end
        end
        
        %% 1. High-Precision Numerical Shooting Solver
        function sol = solveNumericalShooting(obj, munition_prototype, launcher_pos, v0, target_bus, env, branch)
            % Iteratively integrates the vehicle's true Aero/Motor/Gravity physics
            % to find the exact QE (Elevation) and Azimuth to hit target_bus.
            if nargin < 7, branch = 'Low'; end
            
            r_tgt = target_bus.pos;
            dx = r_tgt(1) - launcher_pos(1);
            dy = r_tgt(2) - launcher_pos(2);
            R_horiz = sqrt(dx^2 + dy^2);
            
            % Initial guesses: Azimuth straight at target, Elevation from vacuum formula
            psi = atan2(dy, dx);
            if strcmpi(branch, 'High')
                theta = deg2rad(65.0);
            else
                theta = deg2rad(25.0);
            end
            
            sol.IsValid = false;
            d_theta = deg2rad(0.05); % Perturbation step for finite differences
            d_psi   = deg2rad(0.05);
            
            for iter = 1:obj.MaxIterations
                % Nominal trajectory shot
                [r_nom, tof] = obj.simulateFastBallistic(munition_prototype, launcher_pos, v0, theta, psi, env, r_tgt(3));
                err_x = r_nom(1) - r_tgt(1);
                err_y = r_nom(2) - r_tgt(2);
                miss_dist = sqrt(err_x^2 + err_y^2);
                
                if miss_dist < obj.ToleranceMeters
                    sol.IsValid = true;
                    break;
                end
                
                % Numerical Jacobian perturbations
                [r_pert_th, ~] = obj.simulateFastBallistic(munition_prototype, launcher_pos, v0, theta + d_theta, psi, env, r_tgt(3));
                [r_pert_ps, ~] = obj.simulateFastBallistic(munition_prototype, launcher_pos, v0, theta, psi + d_psi, env, r_tgt(3));
                
                J11 = (r_pert_th(1) - r_nom(1)) / d_theta; % dx/dtheta
                J12 = (r_pert_ps(1) - r_nom(1)) / d_psi;   % dx/dpsi
                J21 = (r_pert_th(2) - r_nom(2)) / d_theta; % dy/dtheta
                J22 = (r_pert_ps(2) - r_nom(2)) / d_psi;   % dy/dpsi
                
                detJ = J11 * J22 - J12 * J21;
                if abs(detJ) < 1e-6
                    break; % Singularity
                end
                
                % Newton-Raphson correction step
                delta_th = ( J22 * err_x - J12 * err_y) / detJ;
                delta_ps = (-J21 * err_x + J11 * err_y) / detJ;
                
                theta = theta - delta_th;
                psi   = psi   - delta_ps;

                % Clamp to mechanical limits
                theta = max(min(theta, obj.ElevationMax), obj.ElevationMin);
                psi   = max(min(psi, obj.AzimuthMax), obj.AzimuthMin);
            end

            if (theta < obj.ElevationMin) || (theta > obj.ElevationMax) ...
                    || (psi < obj.AzimuthMin) || (psi > obj.AzimuthMax)
                sol.IsValid = false; % converged onto a mechanical stop, not a real solution
            end

            sol.Elevation    = theta;
            sol.Azimuth      = psi;
            sol.TimeOfFlight = tof;
            sol.AimPoint     = r_tgt;
            sol.MissDistance = miss_dist;
            sol.Iterations   = iter;
        end
        
        %% 2. Fast Analytical Solvers (With Coordinate Frame Option 2 Applied)
        function sol = solveFiringAngles(obj, launcher_pos, v0, target_bus, mode, branch)
            if nargin < 5, mode = 'Indirect'; end
            if nargin < 6, branch = 'Low'; end
            
            r_rel = target_bus.pos - launcher_pos;
            v_tgt = target_bus.vel;
            
            switch upper(mode)
                case 'INDIRECT'
                    sol = obj.solveBallisticIndirect(r_rel, v0, branch);
                case 'DIRECTLEAD'
                    sol = obj.solveDirectLead(r_rel, v_tgt, v0);
                otherwise
                    error('Unsupported mode: %s.', mode);
            end
            
            if sol.IsValid
                if (sol.Elevation < obj.ElevationMin) || (sol.Elevation > obj.ElevationMax)
                    sol.IsValid = false;
                end
                if (sol.Azimuth < obj.AzimuthMin) || (sol.Azimuth > obj.AzimuthMax)
                    sol.IsValid = false;
                end
            end
        end

        %% 3. Direct Fire (no solving - wraps caller-specified angles in
        %% the same sol shape solveNumericalShooting/solveFiringAngles
        %% return, radians like everywhere else in this class, so a runner
        %% can swap between indirect (FCC-computed) and direct (specified)
        %% firing by changing only which of these three methods it calls -
        %% buildLaunchState/Simulation/reporting downstream stay identical.
        %% TimeOfFlight/AimPoint/MissDistance aren't computed (no target-
        %% seeking happened) - a direct-fire runner picks its own
        %% Simulation TimeSpan rather than deriving it from sol.TimeOfFlight.
        function sol = solveDirect(obj, elevation, azimuth)
            sol.IsValid = (elevation >= obj.ElevationMin) && (elevation <= obj.ElevationMax) ...
                && (azimuth >= obj.AzimuthMin) && (azimuth <= obj.AzimuthMax);
            sol.Elevation    = elevation;
            sol.Azimuth      = azimuth;
            sol.TimeOfFlight = NaN;
            sol.AimPoint     = [NaN; NaN; NaN];
            sol.MissDistance = NaN;
        end

        %% 4. Indirect Fire with Automatic Charge Selection
        function [sol, chosen_idx] = solveNumericalShootingAutoCharge(obj, munition_candidates, launcher_pos, target_bus, env, branch)
            % munition_candidates: cell array of Munition objects, each
            % already built on a specific Charge (e.g.
            % {MunitionFactory.createM107Artillery('Charge5'), ...
            %  MunitionFactory.createM107Artillery('Charge6'), ...
            %  MunitionFactory.createM107Artillery('Charge7')}),
            % ORDERED LOWEST TO HIGHEST muzzle velocity. Tries each in
            % order via solveNumericalShooting and returns the first that
            % reaches the target - standard artillery doctrine of using
            % the lowest sufficient charge (less barrel wear, tighter
            % dispersion, steeper/more concealed trajectory) rather than
            % always maxing out. chosen_idx is 0 (sol.IsValid false) if no
            % candidate reaches the target.
            if nargin < 6, branch = 'Low'; end
            for i = 1:numel(munition_candidates)
                m = munition_candidates{i};
                v0 = m.Charge.MuzzleVelocity;
                sol = obj.solveNumericalShooting(m, launcher_pos, v0, target_bus, env, branch);
                if sol.IsValid
                    chosen_idx = i;
                    return;
                end
            end
            chosen_idx = 0;
            sol.IsValid = false;
        end
    end
    
    methods (Access = private)
        %% Fast 3-DoF Ballistic Integrator (coarser step than Simulation,
        %% but same physics - calls munition.Dynamics.computeDerivatives
        %% directly rather than re-deriving it, so pitch actually follows
        %% the flight-path angle the same way the real Simulation does
        %% instead of staying frozen at the launch angle).
        function [impact_pos, t] = simulateFastBallistic(obj, munition, r0, v0, theta, psi, env, z_target)
            dt = 0.01; % 10 ms coarse step is plenty for shooting convergence
            t_max = 90.0;
            integrator = RK4FixedStep();

            % Built via the munition's own buildLaunchState rather than a
            % hand-assembled vector, so this stays correct regardless of
            % whether the attached Dynamics is the plain 8-state model or
            % a spin-augmented 9-state one (or anything else added later).
            X = munition.buildLaunchState(r0, v0, theta, psi);
            X_prev = X;

            deriv_fn = @(tt, XX) obj.evalMunitionDerivatives(munition, tt, XX, env);

            t = 0;
            while t < t_max
                vel = X(4:6);

                % Terminal condition: altitude crosses target altitude while
                % descending. Linearly interpolates between the last two
                % samples to the exact crossing point rather than returning
                % whatever state happens to land on the dt grid - otherwise
                % the reported impact position is quantized by ~(impact
                % speed * dt), which for this dt/velocity is a few meters:
                % coarser than ToleranceMeters, so solveNumericalShooting's
                % Newton-Raphson could get within a couple of meters of the
                % true answer and then orbit it forever, never satisfying
                % its own convergence check.
                if t > 0.5 && X(3) <= z_target && vel(3) < 0
                    z_prev = X_prev(3);
                    if z_prev ~= X(3)
                        frac = (z_prev - z_target) / (z_prev - X(3));
                    else
                        frac = 0.0;
                    end
                    frac = max(0.0, min(1.0, frac));
                    impact_pos = X_prev(1:3) + frac * (X(1:3) - X_prev(1:3));
                    return;
                end

                X_prev = X;
                X = integrator.step(deriv_fn, t, X, dt);
                t = t + dt;
            end
            impact_pos = X(1:3);
        end

        %% Shared derivative evaluation - mirrors Simulation.evalDerivatives'
        %% bypass branch so the fire-control solver and the real Simulation
        %% are always integrating the exact same physics.
        function dX = evalMunitionDerivatives(~, munition, t, X, env)
            [F_aero, M_aero] = munition.Aero.getForcesAndMoments(t, X, env, 0.0);
            if ~isempty(munition.Propulsion)
                [F_prop, M_prop, m_dot] = munition.Propulsion.getThrust(t, env);
            else
                F_prop = [0;0;0]; M_prop = [0;0;0]; m_dot = 0.0;
            end

            forces.Aero          = F_aero;
            forces.Propulsion    = F_prop;
            forces.FinDeflection = 0.0;
            moments.Aero         = M_aero;
            moments.Propulsion   = M_prop;
            % Full MassProperties (not just Mass) - Dynamics3DofSpin's spin
            % derivative needs Inertia(1,1) too, and reconstructing a
            % partial struct here previously dropped it.
            mass_prop            = munition.MassProperties;

            dX = munition.Dynamics.computeDerivatives(t, X, forces, moments, mass_prop, m_dot);
        end
        
        %% Analytical Direct Lead with Option 2 (Z-Up Frame)
        function sol = solveDirectLead(obj, r_rel, v_tgt, v0)
            sol = struct('IsValid', false, 'Elevation', 0, 'Azimuth', 0, 'TimeOfFlight', 0, 'AimPoint', [0;0;0]);
            
            a = dot(v_tgt, v_tgt) - v0^2;
            b = 2 * dot(r_rel, v_tgt);
            c = dot(r_rel, r_rel);
            
            disc = b^2 - 4 * a * c;
            if disc < 0, return; end
            
            t1 = (-b - sqrt(disc)) / (2 * a);
            t2 = (-b + sqrt(disc)) / (2 * a);
            valid_t = [t1, t2];
            valid_t = valid_t(valid_t > 0);
            if isempty(valid_t), return; end
            t_int = min(valid_t);
            
            % OPTION 2 APPLIED: In Z-Up coordinates, projectile drops down (-Z),
            % so launcher must aim UPWARD (+Z) by 0.5 * g * t^2.
            if strcmpi(obj.CoordFrame, 'Z-Up')
                grav_aim_compensation = [0; 0; 0.5 * obj.Gravity * t_int^2];
            else
                grav_aim_compensation = [0; 0; -0.5 * obj.Gravity * t_int^2];
            end
            
            aim_point = r_rel + v_tgt * t_int + grav_aim_compensation;
            norm_aim = norm(aim_point);
            if norm_aim < 1e-3, return; end
            
            u_aim = aim_point / norm_aim;
            theta = asin(u_aim(3)); % Z-Up elevation
            psi   = atan2(u_aim(2), u_aim(1));
            
            sol.IsValid      = true;
            sol.Elevation    = theta;
            sol.Azimuth      = psi;
            sol.TimeOfFlight = t_int;
            sol.AimPoint     = aim_point;
        end
        
        %% Analytical Ballistic Indirect
        function sol = solveBallisticIndirect(obj, r_rel, v0, branch)
            sol = struct('IsValid', false, 'Elevation', 0, 'Azimuth', 0, 'TimeOfFlight', 0, 'AimPoint', [0;0;0]);
            
            dx = r_rel(1); dy = r_rel(2);
            R_2D = sqrt(dx^2 + dy^2);
            Delta_z = r_rel(3); % Z-Up height
            g = obj.Gravity;
            
            disc = v0^4 - g * (g * R_2D^2 + 2 * Delta_z * v0^2);
            if disc < 0, return; end
            
            if strcmpi(branch, 'HIGH')
                u = (v0^2 + sqrt(disc)) / (g * R_2D);
            else
                u = (v0^2 - sqrt(disc)) / (g * R_2D);
            end
            
            theta = atan(u);
            psi   = atan2(dy, dx);
            
            sol.IsValid      = true;
            sol.Elevation    = theta;
            sol.Azimuth      = psi;
            sol.TimeOfFlight = R_2D / (v0 * cos(theta));
            sol.AimPoint     = r_rel;
        end
    end

    methods (Static)
        function mil = rad2mil(rad)
            % NATO/US artillery mil: 6400 mils per full circle, the unit
            % gun sights and fire-control displays actually read launch
            % angles in - internal solving stays in radians (trig needs
            % it), this is only for reporting a solved/specified launch
            % angle to a person.
            mil = rad * (3200 / pi);
        end

        function rad = mil2rad(mil)
            rad = mil * (pi / 3200);
        end

        function [t_burst, telemetry] = solveBurstTimeForStandoff(munition_builder, launcher_pos, v0, elevation, azimuth, target, env, standoff, timespan, dt, impact_distance, max_downrange)
            % Runs a fresh "recon" flight (its own munition/target
            % instances, so the caller's real round/target are untouched)
            % at the given launch conditions and finds the time the round
            % is `standoff` meters from the (possibly moving) target -
            % the shared basis for both a pre-fire nominal burst-time
            % estimate (fed the charge's nominal muzzle velocity) and a
            % post-fire correction once the round's actual muzzle
            % velocity is measured (call this again with that measured
            % v0 in place of the nominal one - elevation/azimuth stay the
            % same, since those are already fixed by the time muzzle
            % velocity is known).
            %
            % If munition_builder's round has a SelfDestructTime set, the
            % solved burst time is capped at it: a fuze can't be
            % programmed later than the round's own self-destruct ceiling
            % (Simulation.checkTermination enforces that ceiling in flight
            % regardless, but there's no reason to solve for a burst time
            % that would just get overridden anyway).
            if nargin < 11 || isempty(impact_distance), impact_distance = min(2.0, standoff / 2); end
            if nargin < 12 || isempty(max_downrange), max_downrange = 100.0; end

            recon_round = munition_builder();
            recon_round.Dynamics.InitialState = recon_round.buildLaunchState(launcher_pos, v0, elevation, azimuth);
            recon_target = TargetConstantVelocity(target.Position, target.Velocity);

            recon_sim = Simulation(recon_round, recon_target, env, timespan, dt);
            recon_sim.Termination.ImpactDistance = impact_distance;
            recon_sim.Termination.MaxDownrange   = max_downrange;
            recon_tel = recon_sim.run();

            t_burst = FireControlComputer.findRangeCrossingTime(recon_tel, standoff);
            if isa(recon_round, 'Munition') && ~isempty(recon_round.SelfDestructTime)
                t_burst = min(t_burst, recon_round.SelfDestructTime);
            end
            if nargout > 1
                telemetry = recon_tel;
            end
        end

        function t_burst = findRangeCrossingTime(telemetry, standoff)
            % Finds the (interpolated) time at which telemetry.range last
            % crosses down through `standoff` meters - i.e. the instant
            % the round is `standoff` meters from the target, still
            % closing toward intercept.
            range_ = telemetry.range;
            time_  = telemetry.time;
            idx = find(range_(1:end-1) >= standoff & range_(2:end) < standoff, 1, 'last');
            if isempty(idx)
                error('FireControlComputer:findRangeCrossingTime', ...
                    'Range never crossed %.1f m during the recon flight (min range reached = %.1f m).', ...
                    standoff, min(range_));
            end
            r0 = range_(idx);   r1 = range_(idx+1);
            t0 = time_(idx);    t1 = time_(idx+1);
            frac = (r0 - standoff) / (r0 - r1);
            t_burst = t0 + frac * (t1 - t0);
        end
    end
end