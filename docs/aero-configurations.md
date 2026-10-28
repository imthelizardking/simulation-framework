# Deployable-wing aerodynamic configurations

`Aero6DoF` can select or blend coefficient datasets for a civilian UAV's
stowed and deployed configurations. Existing single-dataset property-based
loading continues to work.

## Register datasets

Two ready-to-load **synthetic test datasets** are included:
`data/UAV_Test_Stowed.mat` (3 alpha by 3 Mach points) and
`data/UAV_Test_Deployed.mat` (5 alpha by 5 Mach points). They share reference
dimensions and a moment reference, and include `CD`, `CL`, `CA`, `CN`, `CM`,
and roll/pitch/yaw damping coefficients. Their numbers are arbitrary software
fixtures, not flight data. No mass properties or complete vehicle model are supplied.

Run this example from the repository root:

```matlab
example_uav_wing_deployment        % table and plots
samples = example_uav_wing_deployment(false); % headless assertions
```

It compares an instantaneous switch at 5 seconds with a blend from 5 to 7
seconds, at fixed Mach and alpha. To regenerate/edit the datasets, use
`generate_uav_deployment_data`. For your own scenario, load the structs with:

```matlab
stowed_data = FactoryData.load('UAV_Test_Stowed');
deployed_data = FactoryData.load('UAV_Test_Deployed');
reference_area = stowed_data.RefArea;
reference_length = stowed_data.RefLength;
```

Each dataset is a scalar struct with `Mach` and `Alpha` grids (`Alpha` in
radians), and `CD` and `CL` matrices sized `numel(Alpha)` by `numel(Mach)`.
Grids must be strictly increasing. Other coefficient fields use the same
matrix layout and default to zero when omitted. A singleton grid is allowed.
For the existing body-axis force path, supply `CA` and `CN` as well: `CD`/`CL`
alone do not produce axial/normal body forces.

```matlab
aero = Aero6DoF();
% Supply measured or otherwise justified reference dimensions first.
aero.RefArea = reference_area;
aero.RefLength = reference_length;

% User-supplied structs, or Aero6DoF objects holding coefficient properties:
aero.setConfiguration('Stowed', stowed_data);
aero.setConfiguration('WingsOpen', deployed_data);
aero.ActiveTableKey = 'Stowed';
aero.DeployedTableKey = 'WingsOpen';
aero.DeploymentTime = 5;       % simulation time, seconds
aero.DeploymentDuration = 0;   % switch at t >= 5
```

`setConfiguration` copies the data; subsequent changes to a source object do
not change the registered dataset. It does not select the dataset automatically.
Both datasets must use the same reference area, reference length, coefficient
conventions, and moment reference point. If `RefArea` or `RefLength` appears
in the source, it is checked against the receiving object. The moment reference
point and conventions must be checked by the caller. Keep the receiving
object's reference dimensions fixed after registration.

## Gradual or externally controlled deployment

```matlab
aero.DeploymentDuration = 2;   % stowed through t=5, fully deployed at t=7
c = aero.evaluateCoefficients(mach, alpha, 6); % halfway blend

% Alternatively, drive the ratio from an external deployment model:
aero.DeploymentTime = Inf;    % disable time scheduling (the default)
aero.DeploymentRatio = 0.4;   % 0 = stowed; 1 = fully deployed
c = aero.evaluateCoefficients(mach, alpha);
```

`getForcesAndMoments` passes time automatically. Direct coefficient queries
must pass time to use the schedule; queries without time use the manual ratio.
An empty `DeployedTableKey` disables deployment. To select a named dataset
directly, set `ActiveTableKey` and leave `DeployedTableKey` empty. With the
default `ActiveTableKey = 'Clean'` and no `Tables.Clean`, the original coefficient
properties supply the initial configuration, including when a deployed dataset
has been registered.

Each dataset is interpolated and clamped on its own grid before blending, so
the grids need not match. All coefficient fields, including static moments and
rate derivatives, participate in the blend. This does not add new force or
moment terms to the existing equations.

Linear blending is an explicit approximation, not a validated model of the
deployment transient. For an instantaneous switch, align the event with the
simulation time grid and assess timestep sensitivity. Wing motion, deployment
loads, changes to mass/CG/inertia, rotor thrust, and spray/seed release require
separate models. No existing vehicle presets are changed.

## UAV flight and seed-planting scenario

For a complete illustrative flight/drop sequence in the style of the existing
scenario runners:

```matlab
runner_uav_seed_planting              % flight, seed paths, planting zone
result = runner_uav_seed_planting(false); % same run without plots
```

Edit the settings at the top of `runner_uav_seed_planting.m`: initial position
and speed, planting center/radius, deployment time/duration, release time, and
seed mass, area, drag, count, and dispenser velocity spread. The defaults now use
the standalone `AeroCivilianUAV` model from `createCivilianUAVExampleAero`,
an unpowered 6-DoF glide starting at
40 m, wing deployment from 0.5 to 1 second, and a seed release at 1.5 seconds.
`TargetStatic` represents the planting center; the radius defines a ground zone.

The runner interpolates aircraft position/velocity at release, converts NED to
Z-Up, and drops independent seed point masses using the shared RK4 integrator.
Seeds inherit the UAV velocity with small repeatable dispenser variations.
Ground crossings are interpolated to altitude zero. Returned results include
flight telemetry, release state, seed paths, landing coordinates/times, and
which seeds land inside the planting zone. Seeds not landed by the time limit
are reported separately and have NaN landing coordinates.

This is an open-loop software example, not a trimmed or validated agricultural
aircraft model. Seed flight uses constant ground-level wind/density, quadratic
drag, and gravity. The single-container model updates aircraft mass, CG and
inertia as seeds are released. Planting-zone placement and release time are user inputs;
autonomous navigation and release scheduling are not modeled. The earlier
shared state-layout and coordinate-convention concerns still need separate review.
See [civilian UAV aero](civilian-uav-aero.md) for its independent coefficient
schema. The two `.mat` fixtures above remain for the original table example.

## Verification

From the repository root in MATLAB:

```matlab
addpath(pwd, fullfile(pwd, 'tests'));
results = runtests('tests/TestAeroConfigurations.m');
assert(all([results.Passed]));
```

The tests use synthetic coefficients only. They cover legacy loading, switch
boundaries, blending across different grids, damping derivatives, manual control,
clamping, evaluation order, body-force routing, invalid data, and snapshots.
