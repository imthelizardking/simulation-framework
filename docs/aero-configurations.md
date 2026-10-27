# Deployable-wing aerodynamic configurations

`Aero6DoF` can select or blend coefficient datasets for a civilian UAV's
stowed and deployed configurations. Existing single-dataset property-based
loading continues to work.

## Register datasets

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
