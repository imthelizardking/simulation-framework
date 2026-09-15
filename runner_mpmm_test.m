%% Scenario: DynamicsMPMM/AeroMPMM vs. the Existing M107 Model (Direct Fire)
% Fires the SAME 155mm M107 shell (same mass/inertia/reference dims/CD0/
% CLA/CSPIN/CMAG/CMA/twist) through two different physics models at the
% same launch conditions, so you can see what the new high-fidelity MPMM
% pair (AeroMPMM + DynamicsMPMM - see those files) adds on top of the
% already-working Dynamics3DofSpin + AeroTabulatedSpin model:
%   - "Legacy" model: constant spin axis along velocity, no yaw-of-repose
%     feedback into force, flat constant gravity, no Coriolis.
%   - "MPMM" model: real yaw-of-repose vector (alpha_e), real inverse-
%     square/latitude-corrected gravity, optional Earth-rotation
%     Coriolis - see AeroMPMM.m/DynamicsMPMM.m's class comments for the
%     equations themselves.
% Expect the two trajectories to roughly agree on range/apogee (same
% drag-dominated shape) while the MPMM run shows a small extra lateral
% Deflection the legacy model can't produce at all - that deflection
% (spin-drift from yaw of repose, plus Coriolis) is exactly the new
% physics this comparison is meant to surface.
clear; clc; close all;

charge_name   = 'Charge7'; % edit: 'Charge5' | 'Charge6' | 'Charge7'
elevation_mil = 600;       % edit: quadrant elevation (NATO mils, 6400/circle)
azimuth_mil   = 0.0;       % edit: aim bearing
latitude_deg  = 39.0;      % edit: launch latitude - only read by the MPMM run (real gravity/Coriolis)

env = Environment();
launcher_pos = [0; 0; 0];

%% 1. Legacy model: MunitionFactory's existing M107 preset
legacy = MunitionFactory.createM107Artillery(charge_name);
legacy.Dynamics.Frame = 'NED'; % must be set before buildLaunchState below - it reads this at build time
legacy.Aero.Frame     = 'NED';
v0 = legacy.Charge.MuzzleVelocity;

fcc = FireControlComputer(env.Gravity, 'Z-Up');
sol = fcc.solveDirect(elevation_mil, azimuth_mil);
if ~sol.IsValid
    error('Elevation/azimuth outside the fire control computer''s mechanical limits.');
end
elevation = FireControlComputer.mil2rad(sol.Elevation);
azimuth   = FireControlComputer.mil2rad(sol.Azimuth);

legacy.Dynamics.InitialState = legacy.buildLaunchState(launcher_pos, v0, elevation, azimuth);

sim_legacy = Simulation(legacy, [], env, [0 90], 0.005);
sim_legacy.Frame = 'NED';

fprintf('Firing M107 (legacy model) on %s charge (%.0f m/s) at QE=%.1f mils...\n', charge_name, v0, elevation_mil);
tel_legacy = sim_legacy.run();

%% 2. MPMM model: manually-assembled Munition, same mass/inertia/aero
% coefficients as MunitionFactory.createM107Artillery (see that preset)
% so the comparison isolates the physics difference, not a data
% difference.
mpmm = Munition('M107_155mm_MPMM');
mpmm.Type = 'HE';
mpmm.MassProperties.Mass = legacy.MassProperties.Mass;
mpmm.MassProperties.Inertia = legacy.MassProperties.Inertia;
mpmm.TwistRate = legacy.TwistRate;
mpmm.Charge    = legacy.Charge;

mpmm.Dynamics = DynamicsMPMM();
mpmm.Dynamics.InitLatitudeDeg = latitude_deg;
mpmm.Dynamics.InitAzimuth     = 0.0; % assume local X points true North
mpmm.Dynamics.SimCoriolis     = true;

aero = AeroMPMM();
aero.RefArea   = 0.01887;
aero.RefLength = 0.155;
aero.Ix        = legacy.MassProperties.Inertia(1,1);
aero.LatitudeDeg = latitude_deg;
aero.CD0   = 0.22;
aero.CLA   = 2.5;
aero.CSPIN = -0.03;
aero.CMA   = 2.0;
aero.CMAG  = 1.2;
% Form factors come from the charge itself (see ChargeZone.IForm/FL/QD/
% QM and ChargeFactory.applyFormFactors) - mpmm.Charge was already
% copied from legacy.Charge above.
aero.IForm = mpmm.Charge.IForm;
aero.FL    = mpmm.Charge.FL;
aero.QD    = mpmm.Charge.QD;
aero.QM    = mpmm.Charge.QM;
mpmm.Aero = aero;

mpmm.Dynamics.InitialState = mpmm.buildLaunchState(launcher_pos, v0, elevation, azimuth);

sim_mpmm = Simulation(mpmm, [], env, [0 90], 0.005);
sim_mpmm.Frame = 'NED';

fprintf('Firing M107 (MPMM model) on %s charge (%.0f m/s) at QE=%.1f mils, latitude=%.1f deg...\n', ...
    charge_name, v0, elevation_mil, latitude_deg);
tel_mpmm = sim_mpmm.run();

%% 3. Side-by-Side Comparison
report('Legacy (Dynamics3DofSpin + AeroTabulatedSpin)', tel_legacy, sim_legacy);
report('MPMM (DynamicsMPMM + AeroMPMM)', tel_mpmm, sim_mpmm);

function report(name, tel, sim)
    [apogee, apogee_idx] = max(CoordinateFrame.altitudeColumn(tel.pos(:,3), tel.Frame));
    forward_range = tel.pos(end, 1);
    deflection    = tel.pos(end, 2);
    total_range   = hypot(forward_range, deflection);
    fprintf('\n--- %s ---\n', name);
    fprintf('Termination Reason : %s\n', sim.TerminationReason);
    fprintf('Apogee              : %.1f m at t=%.2f s\n', apogee, tel.time(apogee_idx));
    fprintf('Forward Range       : %.0f m\n', forward_range);
    fprintf('Deflection          : %.0f m\n', deflection);
    fprintf('Total Range         : %.0f m\n', total_range);
    fprintf('Time of Flight      : %.2f s\n', tel.time(end));
end
