# Simulation Framework

A MATLAB object-oriented framework for simulating gun-launched and rocket-propelled
munitions — from muzzle to impact — against static or maneuvering targets. It covers
fire-control solving (quadrant elevation / time-of-flight), 3-to-7 DoF flight dynamics,
aerodynamics (including spin-stabilized and base-bleed effects), guidance/navigation/control
(GNC) for terminal-guided rounds, and post-flight visualization/analysis.

The project models real weapon systems (e.g. 155mm M107 and MOD274 base-bleed artillery,
35mm/76mm AA guns, APKWS-style laser-guided rockets, fin-stabilized darts) as a way to
exercise the framework end-to-end, but the classes are designed to be reused for arbitrary
new munitions, platforms, and engagement scenarios.

> Physical constants and aero coefficients used in the built-in presets are illustrative
> (typical order-of-magnitude values), not validated test data.

## Architecture

The framework is built around a set of `Base*` abstract classes, each with one or more
concrete implementations selected per scenario:

| Concern | Base class | Implementations |
|---|---|---|
| Flight dynamics | `BaseDynamics` | `Dynamics3Dof`, `Dynamics3DofSpin`, `Dynamics6DoF`, `Dynamics7DoF`, `DynamicsMPMM` |
| Aerodynamics | `BaseAero` | `Aero3DoF_Longitudinal`, `AeroTabulated`, `AeroTabulatedSpin` |
| Propulsion | `BasePropulsion` | `Propulsion_SolidMotor`, `Propulsion_DYU` |
| Guidance | `BaseGuidance` | `OptimalAngleFOVGuidance`, `SeekerLOAL` |
| Control | `BaseController` | `LongitudinalFinController` |
| Sensors | `BaseSensor` | `OffboardRadar`, `DataLinkReceiver` |
| Targets | `BaseTarget` | `StaticTarget`, `ConstantVelocityTarget` |
| Integration | `BaseIntegrator` | `RK4FixedStep`, `RK4AdaptiveNearEncounter` |
| Visualization | `BaseVisualizer` | `LiveEngagementPlotter` |

Everything is wired together by [`Simulation.m`](Simulation.m), which steps the target,
designator, ground station, and munition GNC/dynamics in lockstep, logs telemetry each
step, and stops on the first tripped termination criterion (impact, altitude bounds,
max downrange, or max time).

[`FireControlComputer.m`](FireControlComputer.m) solves for the launch elevation/azimuth
and predicted time-of-flight needed to hit a given target (numerical shooting solver over
the actual aero/propulsion/gravity model), which most runner scripts use to set up the
initial launch state before handing off to `Simulation`.

Complete weapon systems are assembled via two factories:
- [`MunitionFactory.m`](MunitionFactory.m) — builds a projectile/rocket (dynamics + aero +
  propulsion + charge + guidance stack), e.g. `createM107Artillery`, `createMOD274`,
  `create35mmHE`, `create76mmHE`, `createAPKWS`, `createFinStabilizedDart`.
- [`WeaponPlatformFactory.m`](WeaponPlatformFactory.m) — builds the launching platform
  (towed howitzer, self-propelled gun, AA mount, naval mount).

Factory presets can be overridden without touching code by dropping a JSON file in
[`data/`](data) named after the factory method (see [`FactoryData.m`](FactoryData.m)),
e.g. `data/createM107Artillery.json` overrides that preset's hardcoded defaults.

## Example scenarios

Each `runner_*.m` script is a self-contained scenario: build an `Environment`, a `Target`,
a munition (via `MunitionFactory`), solve or specify the launch conditions, run the
`Simulation`, and plot the result.

| Script | Scenario |
|---|---|
| [`runner_m107.m`](runner_m107.m) | 155mm M107 unguided HE artillery |
| [`runner_mod274.m`](runner_mod274.m) | 155mm MOD274 base-bleed HE, extended range via `FireControlComputer` |
| [`runner_25mm.m`](runner_25mm.m) | 25mm M793 unguided round |
| [`runner_35mm_burst.m`](runner_35mm_burst.m) | 35mm HE burst fire |
| [`runner_35mm_antidrone.m`](runner_35mm_antidrone.m) | 35mm AA engagement vs. a drone target |
| [`runner_76mm.m`](runner_76mm.m) | 76mm naval HE |
| [`runner_fin_stabilized_dart.m`](runner_fin_stabilized_dart.m) | Fin-stabilized kinetic dart |
| [`runner_sam_engagement.m`](runner_sam_engagement.m) | Laser/seeker-guided intercept engagement |

Run any of them directly in MATLAB from the repo root, e.g.:

```matlab
runner_mod274
```

## Requirements

MATLAB (no toolboxes beyond base MATLAB are required by the core framework).

## Status

Active development — expect interfaces on the `Base*` classes to evolve as new dynamics
models, aero tables, and guidance laws are added.
