# Standalone civilian UAV aerodynamics

`AeroCivilianUAV < BaseAero` is an independent low-speed fixed-wing model.
It plugs into the existing `Munition`, `Dynamics6DoF`, `Simulation`, and mass
properties interfaces. It does not load or translate DATCOM coefficient sets.
No existing aero class, parser, shared dynamics, or vehicle factory is modified.

## Run the civilian example

```matlab
runner_uav_seed_planting
% Headless flight, deployment, and seed release:
result = runner_uav_seed_planting(false);
```

The runner uses `createCivilianUAVExampleAero` for two synthetic stowed/deployed
configurations. Edit that function to explore the coefficient inputs. Its
numbers are software examples, not validated data for an agricultural aircraft.
The earlier `.mat` fixtures and `example_uav_wing_deployment` remain separate
examples of the original configuration-table interface.

## Attach to a vehicle

```matlab
vehicle.Aero = createCivilianUAVExampleAero();
vehicle.Aero.DeploymentTime = 0.5;
vehicle.Aero.DeploymentDuration = 0.5;
% Existing flight setup must use Dynamics6DoF and sim.Frame = 'NED'.
```

For custom data, `Coefficients` supplies the initial `Clean` configuration.
Alternatively, register partial coefficient structs with
`setConfiguration('Stowed', data)` and select `ActiveTableKey`. Use
`DeployedTableKey` to choose the second configuration. With `DeploymentTime=Inf`,
the inherited `DeploymentRatio` supplies manual deployment (0 to 1).
All configurations share one reference area, chord, span, and moment reference.
Blending coefficient parameters is an approximation of the deployment transient.

`RefArea` is reference area in m^2; `RefLength` is mean aerodynamic chord in m;
`Span` is wing span in m. Moments use span for roll/yaw and chord for pitch.
All angle derivatives use radians. Each coefficient is a scalar or a
Mach-dependent scalar representation supported by `BaseAero.evalCoeff`.
Missing coefficient terms are zero. Unknown field names produce an error.

| Contribution | Coefficient fields |
|---|---|
| Profile and additional drag | `CD0`, `CD_alpha2`, `CD_beta2`, `CD_elevator2` |
| Induced drag | `InducedDrag` multiplying total lift coefficient squared |
| Lift | `CL0`, `CL_alpha`, `CL_q`, `CL_alphaDot`, `CL_elevator` |
| Side force | `CY_beta`, `CY_p`, `CY_r`, `CY_rudder` |
| Roll moment | `Cl_beta`, `Cl_p`, `Cl_r`, `Cl_aileron`, `Cl_rudder` |
| Pitch moment | `Cm0`, `Cm_alpha`, `Cm_q`, `Cm_alphaDot`, `Cm_elevator` |
| Yaw moment | `Cn_beta`, `Cn_p`, `Cn_r`, `Cn_aileron`, `Cn_rudder` |

This is a derivative model: the static lift/pitch baselines are constants,
not full-alpha tables to which slopes are added a second time. There is no
center-of-pressure-derived extra moment; moments are defined about the supplied
`MomentReference` and the shared mass pipeline transfers them to current CG.
Keep inherited `IForm` and `FL` at 1; configure drag/lift through these fields.

## Controls and alpha-dot

`getForcesAndMoments(t, X, env, controls)` accepts an elevator angle scalar,
or a struct with optional `Elevator`, `Aileron`, `Rudder` (rad) and `AlphaDot`
(rad/s). The existing controller bus carries this through `FinDeflection`.
The synthetic example flies without an autopilot or control inputs.

Alpha-dot derivatives require `controls.AlphaDot` whenever they are nonzero
at nonzero airspeed. Supplying no value raises an error. The module does not
finite-difference prior aero calls, which would be unsafe under RK4 evaluation.
An external flight/unsteady model must provide a consistent alpha-dot input;
an automatic coupled unsteady solution is not implemented.

## Use independently of the framework state

```matlab
aero = AeroCivilianUAV();
aero.Coefficients = struct('CD0',0.06,'CL_alpha',3,'Cm_q',-2);
velocityBody = [18;0;1];        % air-relative, forward/right/down, m/s
bodyRates = [0;0.1;0];          % p, q, r, rad/s
[F, M, diagnostics] = aero.bodyLoads(0, velocityBody, bodyRates, 1.225, 340);
```

Returned forces are body-axis newtons and moments are body-axis N m. The
wind basis makes drag oppose air-relative velocity and makes lift/side force
perpendicular to it. The framework adapter subtracts NED wind, normalizes the
stored quaternion, and follows the current `Dynamics6DoF` force-path matrix
convention. This is API/force-path compatibility; it does not validate the
legacy quaternion, pitch-rate, or launch-angle conventions. `bodyLoads` avoids
those conventions by accepting body-frame inputs directly.

The model is a small-angle, low-speed approximation. It has no stall,
propeller/rotor aerodynamics, ground effect, aeroelasticity, slosh, or deployment
mechanism loads. Synthetic data and numerical tests do not establish flight
accuracy. The separate mass model continues to own payload mass/CG/inertia.

## Verify

```matlab
addpath(pwd, fullfile(pwd,'tests'));
results = runtests({'tests/TestCivilianUAVAero.m','tests/TestUavSeedPlanting.m'});
assert(all([results.Passed]));
```

Tests cover load scaling, aerodynamic power, sideslip symmetry, damping,
control/rate inputs, explicit unsteady inputs, deployment, Mach interpolation,
wind subtraction, quaternion normalization, and integration with the shared
vehicle/simulation interfaces. MATLAB runtime results must be checked in an
environment with MATLAB installed.
