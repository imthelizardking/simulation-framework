%% File: FireControlComputer.m
classdef FireControlComputer < handle
    properties
        Gravity      (1,1) double = 9.80665
        % ElevationMin/Max/AzimuthMin/Max, and every solve*/solveDirect
        % Elevation/Azimuth input or sol.Elevation/sol.Azimuth output, are
        % in NATO mils (6400/circle - see rad2mil/mil2rad below). This is
        % the "launch angle and its peripherals" boundary only - internal
        % trig (theta/psi inside the solvers, the solver flights,
        % buildLaunchState) still works entirely in radians.
        ElevationMin (1,1) double = FireControlComputer.rad2mil(deg2rad(5.0))   % Launcher physical lower limit
        ElevationMax (1,1) double = FireControlComputer.rad2mil(deg2rad(85.0))  % Launcher physical upper limit
        AzimuthMin   (1,1) double = -3200            % Traverse limit; -3200/3200 = unrestricted (turret/AA mount)
        AzimuthMax   (1,1) double = 3200              % Narrower for a fixed-arc mount (e.g. towed carriage)
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
        
        %% 1. High-Precision Numerical Shooting Solver (static target)
        function sol = solveNumericalShooting(obj, munition_prototype, launcher_pos, v0, target_bus, env, branch, dt, frame)
            % Finds the QE (Elevation) and Azimuth that put the round's
            % descending crossing of the target altitude onto target_bus.pos,
            % by Newton-Raphson over real Simulation flights - the same
            % integrator, step loop and Dynamics/Aero/Propulsion physics the
            % live shot runs, at the same dt/frame (pass the runner's own sim
            % dt and sim.Frame; frame defaults to the round's native one).
            %
            % Each flight is a fresh deep copy of munition_prototype with its
            % Sensors/Guidance/Controller removed (see ballisticTemplate):
            % this is the round's own uncontrolled trajectory, so instance
            % settings (latitude, Coriolis, airbrake_ADT, aero overrides) are
            % all honoured, while no flight can mutate the caller's round or
            % trigger its in-flight GNC (a course-correction solve, a seeker).
            if nargin < 7 || isempty(branch), branch = 'Low'; end
            if nargin < 8 || isempty(dt), dt = 0.005; end
            if nargin < 9 || isempty(frame), frame = FireControlComputer.nativeFrame(munition_prototype); end

            % ElevationMin/Max/AzimuthMin/Max are mils (see class header);
            % the iterative solve below works in radians throughout (feeds
            % buildLaunchState directly), so convert the limits once here
            % rather than repeatedly inside the loop.
            elev_min_rad = FireControlComputer.mil2rad(obj.ElevationMin);
            elev_max_rad = FireControlComputer.mil2rad(obj.ElevationMax);
            azim_min_rad = FireControlComputer.mil2rad(obj.AzimuthMin);
            azim_max_rad = FireControlComputer.mil2rad(obj.AzimuthMax);

            r_tgt = target_bus.pos;
            dx = r_tgt(1) - launcher_pos(1);
            dy = r_tgt(2) - launcher_pos(2);

            % Initial guesses: Azimuth straight at target, Elevation per branch
            psi = atan2(dy, dx);
            if strcmpi(branch, 'High')
                theta = deg2rad(65.0);
            else
                theta = deg2rad(25.0);
            end

            template = FireControlComputer.ballisticTemplate(munition_prototype);
            fly = @(th, ps) FireControlComputer.flyToImpact(template, launcher_pos, v0, ...
                th, ps, env, r_tgt(3), dt, frame);

            sol.IsValid = false;
            d_theta = deg2rad(0.05); % Perturbation step for finite differences
            d_psi   = deg2rad(0.05);
            max_step = deg2rad(10.0); % Newton step limit, guards against overshooting off the branch

            for iter = 1:obj.MaxIterations
                % Nominal trajectory shot
                [r_nom, tof] = fly(theta, psi);
                err_x = r_nom(1) - r_tgt(1);
                err_y = r_nom(2) - r_tgt(2);
                miss_dist = sqrt(err_x^2 + err_y^2);

                if miss_dist < obj.ToleranceMeters
                    sol.IsValid = true;
                    break;
                end

                % Numerical Jacobian perturbations
                r_pert_th = fly(theta + d_theta, psi);
                r_pert_ps = fly(theta, psi + d_psi);

                J11 = (r_pert_th(1) - r_nom(1)) / d_theta; % dx/dtheta
                J12 = (r_pert_ps(1) - r_nom(1)) / d_psi;   % dx/dpsi
                J21 = (r_pert_th(2) - r_nom(2)) / d_theta; % dy/dtheta
                J22 = (r_pert_ps(2) - r_nom(2)) / d_psi;   % dy/dpsi

                detJ = J11 * J22 - J12 * J21;
                if abs(detJ) < 1e-6
                    break; % Singularity
                end

                % Newton-Raphson correction step
                delta = [( J22 * err_x - J12 * err_y) / detJ; ...
                         (-J21 * err_x + J11 * err_y) / detJ];
                if norm(delta) > max_step
                    delta = delta * (max_step / norm(delta));
                end

                theta = theta - delta(1);
                psi   = psi   - delta(2);

                % Clamp to mechanical limits
                theta = max(min(theta, elev_max_rad), elev_min_rad);
                psi   = max(min(psi, azim_max_rad), azim_min_rad);
            end

            if sol.IsValid && ((theta <= elev_min_rad) || (theta >= elev_max_rad) ...
                    || (psi <= azim_min_rad) || (psi >= azim_max_rad))
                sol.IsValid = false; % converged onto a mechanical stop, not a real solution
            end

            sol.Elevation    = FireControlComputer.rad2mil(theta);
            sol.Azimuth      = FireControlComputer.rad2mil(psi);
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

            % solveBallisticIndirect/solveDirectLead are pure-radian
            % analytic solvers (see their own comments); this is the
            % boundary where a solved launch angle becomes mils, the unit
            % ElevationMin/Max/AzimuthMin/Max (and everything downstream:
            % FireMission, buildLaunchState callers) expects.
            if sol.IsValid
                sol.Elevation = FireControlComputer.rad2mil(sol.Elevation);
                sol.Azimuth   = FireControlComputer.rad2mil(sol.Azimuth);
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
        %% return, mils like every other Elevation/Azimuth in this class,
        %% so a runner can swap between indirect (FCC-computed) and direct
        %% (specified) firing by changing only which of these three
        %% methods it calls - buildLaunchState/Simulation/reporting
        %% downstream stay identical (buildLaunchState still wants
        %% radians, so convert with FireControlComputer.mil2rad at that
        %% call site, same as every other solve* caller).
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

        %% 4. Trajectory Shooting Solver (real Simulation flights, moving target)
        function sol = solveTrajectoryShooting(obj, munition_builder, launcher_pos, v0, target, env, timespan, dt, frame)
            % Solves elevation/azimuth by flying the actual model: every
            % candidate is a full Simulation run (fresh munition_builder()
            % round, fresh copy of the target - the caller's own objects
            % are untouched) under the exact Dynamics/Aero/Propulsion/GNC,
            % Frame and dt the real engagement will use, and the miss is
            % read off that flight's true point of closest approach to the
            % (possibly moving) target. Newton-Raphson on the two miss
            % components perpendicular to the closing velocity, with a
            % finite-difference Jacobian from two perturbed flights, drives
            % that miss to zero.
            %
            % Unlike solveDirectLead (vacuum, constant-speed, so drag makes
            % the real round arrive late and low) and solveNumericalShooting
            % (static target only, ground-plane crossing, its own coarser
            % integrator), the converged solution here is the trajectory
            % the real Simulation then reproduces exactly - provided the
            % runner flies it with the same dt/frame passed in here.
            %
            % target is a BaseTarget (its current Position/Velocity are
            % copied into a TargetConstantVelocity for each flight) or a
            % target bus struct (.pos/.vel). timespan bounds each flight.
            % The analytic direct-lead solve is only the initial guess.
            % Returned Elevation/Azimuth are mils like every other solve*;
            % TimeOfFlight is the time of closest approach, AimPoint the
            % target's (Z-Up) position at that time, Telemetry the flight
            % at the returned angles.
            if nargin < 9 || isempty(frame), frame = 'Z-Up'; end

            elev_min_rad = FireControlComputer.mil2rad(obj.ElevationMin);
            elev_max_rad = FireControlComputer.mil2rad(obj.ElevationMax);
            azim_min_rad = FireControlComputer.mil2rad(obj.AzimuthMin);
            azim_max_rad = FireControlComputer.mil2rad(obj.AzimuthMax);

            if isobject(target)
                tgt_bus = target.getTargetBus();
            else
                tgt_bus = target;
            end

            guess = obj.solveDirectLead(tgt_bus.pos - launcher_pos, tgt_bus.vel, v0);
            if guess.IsValid
                theta = guess.Elevation;
                psi   = guess.Azimuth;
            else
                r_rel = tgt_bus.pos - launcher_pos;
                theta = atan2(r_rel(3), hypot(r_rel(1), r_rel(2)));
                psi   = atan2(r_rel(2), r_rel(1));
            end
            theta = max(min(theta, elev_max_rad), elev_min_rad);
            psi   = max(min(psi, azim_max_rad), azim_min_rad);

            d_theta = deg2rad(0.05); % Perturbation step for finite differences
            d_psi   = deg2rad(0.05);
            max_step = deg2rad(5.0); % Newton step limit, guards against a poor initial guess

            fly = @(th, ps) obj.flyToClosestApproach(munition_builder, launcher_pos, v0, ...
                th, ps, tgt_bus, env, timespan, dt, frame);

            sol.IsValid = false;
            for iter = 1:obj.MaxIterations
                ca = fly(theta, psi);
                if ca.MissDistance < obj.ToleranceMeters
                    sol.IsValid = true;
                    break;
                end

                % Miss-plane basis perpendicular to the closing velocity,
                % fixed from the nominal flight so all three flights'
                % misses are measured along the same two axes.
                v_hat = ca.RelVel / norm(ca.RelVel);
                e_h = cross(v_hat, [0; 0; 1]);
                if norm(e_h) < 1e-6, e_h = [0; 1; 0]; end
                e_h = e_h / norm(e_h);
                e_v = cross(e_h, v_hat);
                err = [dot(ca.Miss, e_v); dot(ca.Miss, e_h)];

                ca_th = fly(theta + d_theta, psi);
                ca_ps = fly(theta, psi + d_psi);
                J = [dot(ca_th.Miss, e_v) - err(1), dot(ca_ps.Miss, e_v) - err(1); ...
                     dot(ca_th.Miss, e_h) - err(2), dot(ca_ps.Miss, e_h) - err(2)] ...
                    ./ [d_theta, d_psi];
                if abs(det(J)) < 1e-9
                    break; % Singularity
                end

                delta = J \ err;
                if norm(delta) > max_step
                    delta = delta * (max_step / norm(delta));
                end
                theta = theta - delta(1);
                psi   = psi   - delta(2);

                theta = max(min(theta, elev_max_rad), elev_min_rad);
                psi   = max(min(psi, azim_max_rad), azim_min_rad);
            end
            if ~sol.IsValid
                ca = fly(theta, psi); % report the flight at the final angles, not the pre-update one
                sol.IsValid = ca.MissDistance < obj.ToleranceMeters;
            end

            if (theta <= elev_min_rad) || (theta >= elev_max_rad) ...
                    || (psi <= azim_min_rad) || (psi >= azim_max_rad)
                sol.IsValid = false; % converged onto a mechanical stop, not a real solution
            end
            if ~isempty(ca.SelfDestructTime) && ca.Time > ca.SelfDestructTime
                sol.IsValid = false; % round would self-destruct before reaching the target
            end

            sol.Elevation    = FireControlComputer.rad2mil(theta);
            sol.Azimuth      = FireControlComputer.rad2mil(psi);
            sol.TimeOfFlight = ca.Time;
            sol.AimPoint     = ca.TargetPos;
            sol.MissDistance = ca.MissDistance;
            sol.Iterations   = iter;
            sol.Telemetry    = ca.Telemetry;
        end

        %% 5. Indirect Fire with Automatic Charge Selection
        function [sol, chosen_idx] = solveNumericalShootingAutoCharge(obj, munition_candidates, launcher_pos, target_bus, env, branch, dt, frame)
            % munition_candidates: cell array of Munition objects, each
            % already built on a specific Charge (e.g.
            % {MunitionFactory.createM107Artillery('CH5W'), ...
            %  MunitionFactory.createM107Artillery('CH6W'), ...
            %  MunitionFactory.createM107Artillery('CH7W')}),
            % ORDERED LOWEST TO HIGHEST muzzle velocity. Tries each in
            % order via solveNumericalShooting and returns the first that
            % reaches the target - standard artillery doctrine of using
            % the lowest sufficient charge (less barrel wear, tighter
            % dispersion, steeper/more concealed trajectory) rather than
            % always maxing out. chosen_idx is 0 (sol.IsValid false) if no
            % candidate reaches the target. dt/frame are passed through to
            % solveNumericalShooting.
            if nargin < 6, branch = 'Low'; end
            if nargin < 7, dt = []; end
            if nargin < 8, frame = []; end
            for i = 1:numel(munition_candidates)
                m = munition_candidates{i};
                v0 = m.Charge.MuzzleVelocity;
                sol = obj.solveNumericalShooting(m, launcher_pos, v0, target_bus, env, branch, dt, frame);
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
        %% One full-physics flight for solveTrajectoryShooting, reduced to
        %% its point of closest approach to the target.
        function ca = flyToClosestApproach(~, munition_builder, launcher_pos, v0, theta, psi, tgt_bus, env, timespan, dt, frame)
            round = munition_builder();
            % Frame must be on Dynamics/Aero before buildLaunchState (it
            % reads Dynamics.Frame at build time - see FireMission).
            if isprop(round.Dynamics, 'Frame'), round.Dynamics.Frame = frame; end
            if isprop(round.Aero, 'Frame'), round.Aero.Frame = frame; end
            % Fly through the whole encounter as a single body - a fuze
            % burst or self-destruct would end the flight (and hand off to
            % a fragment swarm) before closest approach is reached.
            ca.SelfDestructTime = round.SelfDestructTime;
            round.BurstTime = [];
            round.SelfDestructTime = [];
            round.Dynamics.InitialState = round.buildLaunchState(launcher_pos, v0, theta, psi);

            recon_target = TargetConstantVelocity(tgt_bus.pos, tgt_bus.vel);
            recon_sim = Simulation(round, recon_target, env, timespan, dt);
            recon_sim.Frame = frame;
            recon_sim.Termination.PassedTarget = true;
            tel = recon_sim.run();

            % Closest approach: sample with the smallest range, refined
            % over the two adjacent dt segments assuming straight-line
            % relative motion within a step (otherwise the miss would be
            % quantized by closing speed * dt - ~1 m for a gun round at
            % dt = 1 ms, coarser than ToleranceMeters).
            rel = tel.pos - tel.target_pos;
            [~, k] = min(vecnorm(rel, 2, 2));
            best = struct('d', norm(rel(k, :)), 'k', k, 's', 0);
            for k0 = max(k - 1, 1):min(k, size(rel, 1) - 1)
                p0 = rel(k0, :);
                dp = rel(k0 + 1, :) - p0;
                if dot(dp, dp) < eps, continue; end
                s = max(0, min(1, -dot(p0, dp) / dot(dp, dp)));
                d = norm(p0 + s * dp);
                if d < best.d, best = struct('d', d, 'k', k0, 's', s); end
            end
            k0 = best.k;
            k1 = min(k0 + 1, size(rel, 1));
            lerp = @(A) (A(k0, :) + best.s * (A(k1, :) - A(k0, :)))';

            % Back to the Z-Up-sense convention the solver (and target_bus)
            % work in - rotateVector is self-inverse.
            pos     = CoordinateFrame.rotateVector(lerp(tel.pos), frame);
            tgt_pos = CoordinateFrame.rotateVector(lerp(tel.target_pos), frame);
            vel     = CoordinateFrame.rotateVector(lerp(tel.vel), frame);

            ca.Miss         = pos - tgt_pos;
            ca.MissDistance = norm(ca.Miss);
            ca.RelVel       = vel - tgt_bus.vel;
            ca.Time         = tel.time(k0) + best.s * (tel.time(k1) - tel.time(k0));
            ca.TargetPos    = tgt_pos;
            ca.Telemetry    = tel;
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
            % angles in - it's the unit for ElevationMin/Max/AzimuthMin/
            % Max and every solve*/solveDirect Elevation/Azimuth in this
            % class. Internal trig-heavy solving (theta/psi inside the
            % solvers, their Simulation flights, buildLaunchState) stays in
            % radians throughout; this pair converts at that boundary.
            mil = rad * (3200 / pi);
        end

        function rad = mil2rad(mil)
            rad = mil * (pi / 3200);
        end

        function [t_burst, telemetry] = solveBurstTimeForStandoff(munition_builder, launcher_pos, v0, elevation, azimuth, target, env, standoff, timespan, dt, impact_distance, max_downrange, frame)
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
            % elevation/azimuth are mils, same as the solveFiringAngles/
            % solveNumericalShooting/solveDirect sol.Elevation/sol.Azimuth
            % callers typically pass straight through here - converted to
            % radians internally, right before buildLaunchState (which
            % stays radians-only, like every Dynamics-facing call).
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
            % frame (optional) must match the real engagement's sim.Frame -
            % left at Simulation's 'Z-Up' default, an FDR-native round
            % (DynamicsMPMM) reads its crossrange as altitude and the
            % recon trips MinAltitude on its first step.
            if nargin < 13 || isempty(frame), frame = FireControlComputer.nativeFrame(recon_round); end
            if isprop(recon_round.Dynamics, 'Frame'), recon_round.Dynamics.Frame = frame; end
            if isprop(recon_round.Aero, 'Frame'), recon_round.Aero.Frame = frame; end
            recon_round.Dynamics.InitialState = recon_round.buildLaunchState(launcher_pos, v0, ...
                FireControlComputer.mil2rad(elevation), FireControlComputer.mil2rad(azimuth));
            recon_target = TargetConstantVelocity(target.Position, target.Velocity);

            recon_sim = Simulation(recon_round, recon_target, env, timespan, dt);
            recon_sim.Frame = frame;
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

        function [adt, info] = solveDeployTimeForRange(munition_builder, current_state, current_t, desired_range, env, tol, max_iter)
            % Course-correction fuze solve: finds the airbrake_ADT (see
            % AeroMPMM) that brings a round already in flight down to
            % desired_range meters, by bisection over candidate deploy
            % times. Bisects rather than root-finding on a derivative
            % because the relationship is monotonic but not smooth in any
            % convenient closed form: an EARLIER deploy time means MORE of
            % the flight happens under the higher-drag phase-2
            % coefficients, so range decreases monotonically as
            % airbrake_ADT decreases.
            %
            % Deliberately mirrors solveBurstTimeForStandoff's own recon-
            % flight pattern (munition_builder is a caller-supplied
            % closure, called fresh for every candidate) rather than
            % reusing that method directly, for the same reason that
            % method exists: munition_builder must copy EVERY instance-
            % level setting off the real round (launch latitude,
            % SimCoriolis, Aero.LatitudeDeg, all phase-1 AND phase-2 aero
            % coefficients, mass/inertia, twist-rate-derived spin) - a
            % fresh MunitionFactory.createXxx() call alone silently drops
            % all of that (the exact bug this framework hit building
            % solveBurstTimeForStandoff's own recon). Each bisection
            % candidate needs its OWN fresh recon round (not the same one
            % reused) because AeroMPMM's Phase2Active latch is one-shot -
            % reusing one recon object across trials would leave it stuck
            % in phase 2 after the first trial that deploys.
            %
            % current_state/current_t are the REAL round's own state and
            % absolute time-since-launch at the moment this is called
            % (e.g. nav_state/t from Controller_CourseCorrection.step) -
            % every recon flight below starts from there, under exact,
            % unmodified physics, not a fresh from-zero launch.
            %
            % Edge cases (not errors - valid, reported outcomes):
            %   - desired_range is beyond even the "never deploy" recon's
            %     own range -> clamp airbrake_ADT = Inf (never deploy).
            %   - desired_range is closer than even an "immediate deploy"
            %     (airbrake_ADT = current_t) recon achieves -> clamp
            %     airbrake_ADT = current_t (deploy right now).
            % Both are printed, not silently swallowed.
            if nargin < 6 || isempty(tol), tol = 3.0; end          % meters
            if nargin < 7 || isempty(max_iter), max_iter = 30; end

            ground_alt = 0.0; % ground-impact reference - matches every runner's launcher_pos = [0;0;0]

            [range_never, t_impact_never] = FireControlComputer.reconFlightRange( ...
                munition_builder, current_state, current_t, Inf, env, ground_alt);

            if desired_range >= range_never
                adt = Inf;
                info = struct('EdgeCase', 'NeverDeploy', 'AchievedRange', range_never, ...
                    'Iterations', 0, 'RangeAtNeverDeploy', range_never, 'RangeAtImmediateDeploy', NaN);
                fprintf(['[solveDeployTimeForRange] Desired range %.1f m is beyond even the ' ...
                    'no-deploy trajectory (%.1f m) - clamping to airbrake_ADT = Inf (never deploy).\n'], ...
                    desired_range, range_never);
                return;
            end

            [range_immediate, ~] = FireControlComputer.reconFlightRange( ...
                munition_builder, current_state, current_t, current_t, env, ground_alt);

            if desired_range <= range_immediate
                adt = current_t;
                info = struct('EdgeCase', 'DeployImmediately', 'AchievedRange', range_immediate, ...
                    'Iterations', 0, 'RangeAtNeverDeploy', range_never, 'RangeAtImmediateDeploy', range_immediate);
                fprintf(['[solveDeployTimeForRange] Desired range %.1f m is closer than even an ' ...
                    'immediate deploy achieves (%.1f m) - clamping to airbrake_ADT = %.3f s (deploy now).\n'], ...
                    desired_range, range_immediate, current_t);
                return;
            end

            % Bisect over [current_t, t_impact_never] - any candidate at
            % or past t_impact_never is physically equivalent to "never
            % deploy" (the un-braked round has already landed by then), so
            % this is a real, finite stand-in for the Inf upper bound
            % rather than an arbitrary constant.
            lo = current_t;
            hi = t_impact_never;
            adt = 0.5 * (lo + hi);
            range_mid = NaN;
            k = 0;
            for k = 1:max_iter
                adt = 0.5 * (lo + hi);
                [range_mid, ~] = FireControlComputer.reconFlightRange( ...
                    munition_builder, current_state, current_t, adt, env, ground_alt);
                if abs(range_mid - desired_range) <= tol
                    break;
                end
                if range_mid > desired_range
                    hi = adt; % still overshooting -> deploy earlier (more drag time)
                else
                    lo = adt; % undershooting -> deploy later (less drag time)
                end
            end

            info = struct('EdgeCase', 'None', 'AchievedRange', range_mid, 'Iterations', k, ...
                'RangeAtNeverDeploy', range_never, 'RangeAtImmediateDeploy', range_immediate);
        end
    end

    methods (Static, Access = private)
        function template = ballisticTemplate(prototype)
            % Deep copy of prototype (whole handle graph, via MATLAB's own
            % serialization) with Sensors/Guidance/Controller removed - the
            % round's uncontrolled flight, used by solveNumericalShooting.
            % GNC is detached BEFORE copying, both because it isn't flown and
            % because it can hold closures over the prototype itself (e.g.
            % Controller_CourseCorrection.ReconBuilder) that shouldn't be
            % serialized; it's reattached to the prototype straight after.
            gnc = {prototype.Sensors, prototype.Guidance, prototype.Controller};
            prototype.Sensors = {}; prototype.Guidance = []; prototype.Controller = [];
            try
                bytes = getByteStreamFromArray(prototype);
            catch err
                [prototype.Sensors, prototype.Guidance, prototype.Controller] = gnc{:};
                rethrow(err);
            end
            [prototype.Sensors, prototype.Guidance, prototype.Controller] = gnc{:};
            template = bytes;
        end

        function [impact_pos, t_impact] = flyToImpact(template, launcher_pos, v0, theta, psi, env, z_target, dt, frame)
            % One real Simulation flight of a fresh copy of the
            % ballisticTemplate, reduced to where it crosses z_target
            % (height above ground, Z-Up sense) while descending - linearly
            % interpolated between the bracketing dt samples so the impact
            % point isn't quantized by impact speed * dt. impact_pos is in
            % the Z-Up-sense convention target_bus/launcher_pos use.
            round = getArrayFromByteStream(template);
            if isprop(round.Dynamics, 'Frame'), round.Dynamics.Frame = frame; end
            if isprop(round.Aero, 'Frame'), round.Aero.Frame = frame; end
            round.LastGNCBus = struct();
            round.BurstTime = [];
            round.SelfDestructTime = [];
            round.Dynamics.InitialState = round.buildLaunchState(launcher_pos, v0, theta, psi);

            % Ceiling well past any tube-artillery/rocket flight here (the
            % old 90 s cap silently truncated lofted GMLRS shots mid-air).
            max_flight_time = 300.0;
            sim = Simulation(round, [], env, [0 max_flight_time], dt);
            sim.Frame = frame;
            launch_alt = CoordinateFrame.altitude(round.Dynamics.InitialState, frame);
            sim.Termination.MinAltitude = min(launch_alt, z_target) - 1.0;
            tel = sim.run();

            % pos rows back to Z-Up sense (rotateVector is linear and
            % self-inverse, so one 3x3 matrix does the whole series).
            M = [CoordinateFrame.rotateVector([1;0;0], frame), ...
                 CoordinateFrame.rotateVector([0;1;0], frame), ...
                 CoordinateFrame.rotateVector([0;0;1], frame)];
            P = tel.pos * M';
            alt = P(:, 3);
            k = find(alt(1:end-1) >= z_target & alt(2:end) < z_target, 1, 'last');
            if isempty(k)
                % Never came back down through z_target (out of time, or
                % never got that high) - report where it ended up.
                impact_pos = P(end, :)';
                t_impact = tel.time(end);
                return;
            end
            frac = (alt(k) - z_target) / (alt(k) - alt(k+1));
            impact_pos = (P(k, :) + frac * (P(k+1, :) - P(k, :)))';
            t_impact = tel.time(k) + frac * (tel.time(k+1) - tel.time(k));
        end

        function frame = nativeFrame(munition)
            % The frame a munition's Dynamics currently works in - its own
            % Frame property if it has one, otherwise 'FDR' for the always-
            % FDR full-state DynamicsMPMM, else legacy 'Z-Up'.
            if isprop(munition.Dynamics, 'Frame')
                frame = munition.Dynamics.Frame;
            elseif munition.Dynamics.StateDimension == BaseDynamics.FULL_STATE_DIM
                frame = 'FDR';
            else
                frame = 'Z-Up';
            end
        end

        function [range, t_impact] = reconFlightRange(munition_builder, current_state, current_t, adt, env, ground_alt)
            % One recon flight for solveDeployTimeForRange: builds a
            % fresh munition_builder() copy (fresh every call - see that
            % method's comment on why), starts it from the REAL round's
            % current_state/current_t rather than a fresh launch, sets the
            % candidate airbrake_ADT under test, and runs it to ground
            % impact under exact physics (a real Simulation object, same
            % as solveBurstTimeForStandoff's own recon flights - not a
            % degraded/coarser physics path).
            recon = munition_builder();
            if ~isa(recon.Aero, 'AeroMPMM')
                error('FireControlComputer:solveDeployTimeForRange', ...
                    'solveDeployTimeForRange requires an AeroMPMM-based munition (airbrake_ADT lives there).');
            end
            recon.Aero.airbrake_ADT = adt;
            recon.Aero.Phase2Active = false;
            recon.Dynamics.InitialState = current_state;

            % No target needed - only ground-impact termination matters
            % for this recon; Simulation's resolveTargetBus falls back to
            % a harmless zeroed struct when Target is [].
            %
            % max_flight_time is a worst-case ceiling, not tuned per
            % munition - a worst-case ceiling for the same reason
            % flyToImpact uses one (a round-size-specific
            % guess here previously cut M107-class flight times short,
            % silently returning the round's mid-flight position instead
            % of its true impact point - see the TerminationReason check
            % below, added for the same reason: fail loudly instead of
            % ever returning that again).
            max_flight_time = 90.0;
            recon_sim = Simulation(recon, [], env, [current_t, current_t + max_flight_time], 0.002);
            recon_sim.Frame = 'FDR';
            recon_sim.Termination = struct('MaxTime', [], 'ImpactDistance', [], ...
                'MinAltitude', ground_alt, 'MaxAltitude', [], 'MaxDownrange', [], 'PassedTarget', false);

            tel = recon_sim.run();
            if ~strcmp(recon_sim.TerminationReason, 'MinAltitude')
                error('FireControlComputer:solveDeployTimeForRange', ...
                    ['Recon flight (airbrake_ADT candidate = %.3f s) never reached ground impact within ' ...
                     '%.1f s past t=%.3f s (got as far as t=%.2f s, range %.1f m). Increase max_flight_time ' ...
                     'in reconFlightRange.'], ...
                    adt, max_flight_time, current_t, tel.time(end), hypot(tel.pos(end,1), tel.pos(end,3)));
            end
            final_pos = tel.pos(end, :)';
            range = hypot(final_pos(1), final_pos(3));
            t_impact = tel.time(end);
        end
    end
end