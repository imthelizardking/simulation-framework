%% Master Scenario: 155mm M107 Course-Correction Airbrake Fuze (Closed-Loop)
% Full pipeline: fire control aims at Multiplier*Desired_Range (pure
% phase-1 ballistics - airbrake_ADT defaults to Inf/never-deploy, so this
% is just an ordinary overshoot shot), then launches. Once airborne, the
% MuzzleVelocityRadar + PassthroughGuidance + CourseCorrectionController
% chain takes over automatically (see Munition.updateGNC): the moment the
% radar's one-shot reading goes valid (after AcquisitionDelay seconds),
% the controller solves for the airbrake_ADT that brings the round's OWN
% real in-flight state down to Desired_Range - using exact-physics recon
% flights, not degraded ones (see FireControlComputer.
% solveDeployTimeForRange) - and sets it on the live round. AeroMPMM's
% one-shot Phase2 switch does the rest on its own from there.
clear; clc; close all;

Desired_Range = 15000.0; % m - the range the airbrake correction should bring the round down to
Multiplier    = 1.15;    % fire control aims at Multiplier*Desired_Range - the deliberate overshoot the airbrake then corrects away

% 1. Setup Environment
env = Environment();
launcher_pos = [0; 0; 0];
latitude_deg = 39.0; % launch latitude - feeds real gravity/Coriolis; also the setting build_m107_recon (below) must replicate

% 2. Instantiate the real round
shell = MunitionFactory.createM107Artillery('CH7W');
shell.Dynamics.InitLatitudeDeg = latitude_deg;
shell.Dynamics.SimCoriolis     = true;
shell.Aero.LatitudeDeg         = latitude_deg;
v0_nominal = shell.Charge.MuzzleVelocity;

% Phase2 coefficients already come from MunitionFactory.createM107Artillery
% (data/M107_155mm.mat's aero.phase2.* section if present, otherwise an
% illustrative CD0 bump - see that factory method) - nothing to set here.
% airbrake_ADT itself stays at its Inf default - the controller sets a
% finite value later, once airborne.

% 3. Recon-round builder for FireControlComputer.solveDeployTimeForRange -
% see build_m107_recon at the bottom of this file. Must copy every
% instance-level setting off `shell` (latitude, Coriolis, phase-1 AND
% phase-2 aero coefficients, mass/inertia, spin) rather than relying on a
% fresh MunitionFactory.createM107Artillery() call, which would silently
% drop all of that - the exact gotcha solveBurstTimeForStandoff's own
% recon pattern already has to guard against.
recon_builder = @() build_m107_recon(shell, latitude_deg);

% 4. Sensor + Guidance + Controller pipeline (see MuzzleVelocityRadar,
% PassthroughGuidance, CourseCorrectionController)
mvr = MuzzleVelocityRadar();
mvr.Name             = 'MVR';
mvr.AcquisitionDelay = 1.0;
mvr.NoiseStdDev      = 3.0;

ctrl = CourseCorrectionController();
ctrl.ReconBuilder = recon_builder;
ctrl.DesiredRange = Desired_Range;

shell.Sensors    = {mvr};
shell.Guidance   = PassthroughGuidance();
shell.Controller = ctrl;

% 5. Fire Control Computer - aim at Multiplier*Desired_Range using the
% NOMINAL muzzle velocity, exactly like runner_m107.m's own solve.
fcc = FireControlComputer(env.Gravity, 'Z-Up');
aim_target = TargetStatic([Multiplier * Desired_Range; 0; 0], [], 'FDR');

fprintf('Running Fire Control numerical shooting solver (aiming at %.1f m = %.2fx Desired_Range)...\n', ...
    Multiplier * Desired_Range, Multiplier);
sol = fcc.solveNumericalShooting(shell, launcher_pos, v0_nominal, aim_target.getTargetBus(), env, 'Low');

if ~sol.IsValid
    error('Aim point is out of range or solution failed to converge!');
end
fprintf('Quadrant Elevation (QE) : %.1f mils\n', sol.Elevation);
fprintf('Predicted Flight Time   : %.2f s\n', sol.TimeOfFlight);

% 6. Pre-calculated airbrake_ADT: what solveDeployTimeForRange would find
% under the NOMINAL (undispersed) muzzle velocity - i.e. if this shot flew
% exactly as fire control assumed, with zero dispersion. Flown forward to
% AcquisitionDelay with a plain manual RK4 loop (fly_to_time, below) since
% Simulation's own telemetry doesn't retain the full state vector (spin,
% mass) a mid-flight recon start needs - only pos/vel/pitch. Purely for
% comparison against the after-calculated (actual, in-flight-solved) ADT
% reported after the flight below - the gap between the two IS the course-
% correction fuze's whole reason to exist.
fprintf('Solving pre-calculated (nominal-velocity) airbrake_ADT for comparison...\n');
nominal_round = recon_builder();
X_nominal_at_acq = fly_to_time(nominal_round, launcher_pos, v0_nominal, ...
    FireControlComputer.mil2rad(sol.Elevation), 0, env, 0.001, mvr.AcquisitionDelay);
[adt_precalculated, info_precalculated] = FireControlComputer.solveDeployTimeForRange( ...
    recon_builder, X_nominal_at_acq, mvr.AcquisitionDelay, Desired_Range, env);
fprintf('Pre-Calculated Airbrake ADT (nominal v0=%.2f m/s) : %.4f s (edge case: %s, recon range %.1f m)\n', ...
    v0_nominal, adt_precalculated, info_precalculated.EdgeCase, info_precalculated.AchievedRange);

% 7. Actual (dispersed) muzzle velocity for THIS shot - the true value the
% radar's single perturbed reading is based on (see MuzzleVelocityRadar's
% own comment on why it reads this directly rather than back-solving it).
v0_actual = v0_nominal + shell.Charge.MuzzleVelocityStdDev * randn();
mvr.TrueMuzzleVelocity = v0_actual;

shell.Dynamics.InitialState = shell.buildLaunchState(launcher_pos, v0_actual, ...
    FireControlComputer.mil2rad(sol.Elevation), 0);
fprintf('Form Factors (at QE=%.1f mils, zone %s) : IForm=%.4f, FL=%.4f\n', ...
    sol.Elevation, shell.Charge.Name, shell.Aero.IForm, shell.Aero.FL);

fprintf('Nominal muzzle velocity : %.2f m/s\n', v0_nominal);
fprintf('Actual muzzle velocity  : %.2f m/s (this shot''s true dispersion)\n', v0_actual);

% 8. Assemble Simulation & Execute - the sensor/guidance/controller
% pipeline above runs automatically inside Simulation.run() via
% Munition.updateGNC, exactly like any other guided round in this
% framework.
sim = Simulation(shell, [], env, [0 sol.TimeOfFlight * 1.5], 0.005);
sim.Frame = 'FDR';

fprintf('Starting trajectory integration...\n');
tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.2f s simulated).\n', toc, telemetry.time(end));

% 9. Post-Flight Analysis
[apogee, apogee_idx] = max(CoordinateFrame.altitudeColumn(telemetry.pos(:,2), telemetry.Frame));
final_pos = telemetry.pos(end, :)';
impact_range = hypot(final_pos(1), final_pos(3));

fprintf('\n=== Results ===\n');
fprintf('Termination Reason         : %s\n', sim.TerminationReason);
fprintf('Apogee                     : %.1f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Time of Flight             : %.2f s\n', telemetry.time(end));
fprintf('Pre-Calculated ADT (nominal v0)  : %.4f s (edge case: %s)\n', adt_precalculated, info_precalculated.EdgeCase);
if ~isempty(fieldnames(ctrl.SolveInfo))
    fprintf('After-Calculated ADT (actual v0) : %.4f s (edge case: %s)\n', shell.Aero.airbrake_ADT, ctrl.SolveInfo.EdgeCase);
    fprintf('Recon-Predicted Range      : %.1f m\n', ctrl.SolveInfo.AchievedRange);
else
    fprintf('After-Calculated ADT (actual v0) : never solved - MVR reading never went valid during flight\n');
end
fprintf('Desired Range              : %.1f m\n', Desired_Range);
fprintf('Actual Impact Range        : %.1f m\n', impact_range);
fprintf('Range Error                : %.1f m\n', impact_range - Desired_Range);

plotTrajectoryProfile(telemetry, [], sprintf('M107 Course-Correction (Desired Range = %.0f m)', Desired_Range));

%% Local function: exact-physics recon-round builder
function recon = build_m107_recon(real_shell, latitude_deg)
    % Fresh recon Munition replicating every instance-level setting of
    % real_shell that MunitionFactory.createM107Artillery() alone would
    % NOT reproduce - latitude/Coriolis, phase-1 AND phase-2 aero
    % coefficients (including the QE-specific IForm/FL already applied to
    % real_shell by buildLaunchState), mass/inertia, and spin-relevant
    % TwistRate. No Sensors/Guidance/Controller attached - a pure
    % ballistic recon copy, run fresh for every bisection candidate (see
    % FireControlComputer.solveDeployTimeForRange).
    recon = MunitionFactory.createM107Artillery('CH7W');
    recon.Dynamics.InitLatitudeDeg = latitude_deg;
    recon.Dynamics.SimCoriolis     = real_shell.Dynamics.SimCoriolis;
    recon.Aero.LatitudeDeg         = latitude_deg;

    recon.Aero.CD0    = real_shell.Aero.CD0;
    recon.Aero.CDA2   = real_shell.Aero.CDA2;
    recon.Aero.CDA4   = real_shell.Aero.CDA4;
    recon.Aero.CLA    = real_shell.Aero.CLA;
    recon.Aero.CLA3   = real_shell.Aero.CLA3;
    recon.Aero.CLA5   = real_shell.Aero.CLA5;
    recon.Aero.CMA    = real_shell.Aero.CMA;
    recon.Aero.CMA3   = real_shell.Aero.CMA3;
    recon.Aero.CSPIN  = real_shell.Aero.CSPIN;
    recon.Aero.CMAG   = real_shell.Aero.CMAG;
    recon.Aero.IForm  = real_shell.Aero.IForm;
    recon.Aero.FL     = real_shell.Aero.FL;
    recon.Aero.Ix     = real_shell.Aero.Ix;
    recon.Aero.Phase2 = real_shell.Aero.Phase2;

    recon.MassProperties = real_shell.MassProperties;
    recon.TwistRate       = real_shell.TwistRate;

    recon.Sensors    = {};
    recon.Guidance   = [];
    recon.Controller = [];
end

%% Local function: manual exact-physics flight to a given time
function X = fly_to_time(mun, launcher_pos, v0, elevation, azimuth, env, dt, t_end)
    % Flies `mun` from launch to t_end under exact physics (Munition.
    % getDerivatives, the same call every other flight in this framework
    % goes through), returning the FULL final state vector directly -
    % unlike Simulation.run()'s telemetry, which only retains pos/vel/
    % pitch (not spin/mass), this keeps the actual state vector in hand
    % throughout, so it's usable as a recon flight's starting point (see
    % FireControlComputer.solveDeployTimeForRange, which requires the
    % real, complete state - not a partial reconstruction).
    mun.Dynamics.InitialState = mun.buildLaunchState(launcher_pos, v0, elevation, azimuth);
    X = mun.Dynamics.InitialState;
    integrator = RK4FixedStep();
    t = 0;
    while t < t_end - 1e-9
        step_dt = min(dt, t_end - t);
        mun.Dynamics.CurrentState = X;
        X = integrator.step(@(tt, XX) mun.getDerivatives(tt, XX, env), t, X, step_dt);
        t = t + step_dt;
    end
end
