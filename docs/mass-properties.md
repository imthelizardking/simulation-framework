# Shared mass, CG, and inertia evaluation

All native `Munition` simulations now use the same mass-properties pipeline.
Existing runner scripts and factories keep their `MassProperties` input structs;
there is no separate UAV-only dynamics path. The UAV's single container is an
additional input model feeding that shared pipeline.

## Existing inputs

`MassProperties.Mass` is initial/full mass in kg. `Inertia` is a tensor about
the configured CG in body axes, kg m^2. With only those fields, CG defaults
to the body datum and inertia remains constant. Inertia is never silently
replaced by unused full/empty defaults. The struct remains configuration;
`CurrentMassProperties` is the last accepted simulation sample.

Optional endpoint fields remain supported: `MassEmpty`, `CGFull`, `CGEmpty`,
`InertiaFull`, and `InertiaEmpty`. CG positions are meters from a common
body-fixed datum. Missing CG endpoints fall back to a fixed `CG` (or zero).
Missing inertia endpoints fall back to `Inertia`. Endpoint interpolation
assumes uniformly depleted material with a fixed component CG; tensors are
transferred to a common datum before interpolation and then to the current CG.
Endpoints are not enough to model an arbitrarily shaped draining tank.

Three-element inertia vectors and existing 3x3 matrices are supported. Tensors
must be finite, symmetric, positive definite and obey the principal-moment
triangle inequality. Unsupported 4x4 dual-spin matrices are rejected explicitly;
the existing `Dynamics7DoF` is still a stub.

`Dynamics3Dof` (and its spin subclass) reads/writes mass at index 8; full-state
models use index 14. Inertia state entries are derived diagnostics populated
at accepted samples, not independent integrated variables. `DynamicsMPMM`
uses the current state mass, and `AeroMPMM` receives the same inertia snapshot
as its dynamics instead of relying on an independently cached axial inertia.
Its optional legacy base-bleed coefficient/CG-shift model remains separate;
this change does not validate that model's equations.

The solid motor exposes cumulative mass consumption from its existing thrust
table/constant flow so mass does not drift past burnout due to RK4 boundary
sampling. A positive remaining mass and consistent explicitly supplied empty
mass are required. Other propulsion implementations still supply `m_dot`.

## Single UAV container

```matlab
model = UAVMassModel();
model.DryMass = 12;                  % includes the empty container
model.DryCG = [0;0;0];
model.DryInertia = diag([1 2 3]);     % about dry-airframe CG
model.InitialPayloadMass = 2;
model.PayloadCG = [0.2;0;0];
model.PayloadInertiaPerKg = diag([0.02 0.02 0.02]);
model.ReleaseTime = 5;
model.ReleaseDuration = 2;           % 0 for a discrete release
vehicle.MassModel = model;          % the existing Munition container
```

The container is fixed in body axes and its contents are uniformly depleted
at fixed spatial distribution. Mass-weighted CG and parallel-axis component
inertias are calculated directly. Time lookup is pure and bounded, including
out-of-order RK4 calls. Both the seed-planting runner and old scenarios use
`Munition.massPropertiesAt` and `synchronizeMassState` before GNC and logging.
Container and fuel depletion cannot be combined implicitly: supply a combined
model before supporting both simultaneously.

## Moment references and limits

For `Dynamics6DoF`, forces and moment references must be in the same body axes.
The reference-to-CG transfer uses `(reference - currentCG) x force`.
`Aero.MomentReference` and `Propulsion.MomentReference` independently override
the legacy `CGRef` fallback. A reference at the body datum defaults to zero.
No body-CG cross product is applied to an inertial-frame point-mass force.

Full 3x3 inertia is used by the 6-DoF angular acceleration calculation. The
existing attitude/frame conventions, including the pitch-rate sign convention,
remain in place; this change does not establish their physical validity.
Variable-mass motion remains a quasi-static rigid-body approximation: slosh,
discharge recoil/angular-momentum transport, CG migration kinematics and
moving-wing inertial loads require additional modeling. Instantaneous release
does not apply an impulse or change the vehicle velocity. Align discrete
release times with the simulation grid and assess timestep sensitivity.

## Validate with old scenarios

From the repository root in MATLAB:

```matlab
addpath(pwd,fullfile(pwd,'tests'));
results = runtests('tests/TestMassProperties.m');
assert(all([results.Passed]));
report = validateMassPropertiesScenarios;
% Or choose existing scenarios:
report = validateMassPropertiesScenarios({'m107_direct','25mm'});
```

The report executes the existing runners, checks their mass/CG/inertia
telemetry, and compares their metrics with existing saved baselines. Failures
and missing baselines are explicit. A baseline match is a regression check,
not experimental validation. Baselines are not rewritten. Correcting the
previous inertia overwrite, current-mass selection, and reference transfer
can intentionally change old trajectories. The headless metric collector now
selects the altitude column by frame; old Z-Up baselines may also differ in
apogee because the previous collector incorrectly used column 2.

Review differences before deciding whether to update any baseline. Existing
unrelated naming/data/coordinate issues can still prevent a scenario from
running; the report records those errors rather than treating them as passes.
