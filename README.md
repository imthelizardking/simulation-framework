# Simulation Framework

A MATLAB object-oriented framework for simulating gun-launched and rocket-propelled
munitions — from muzzle to impact — against static or maneuvering targets. It covers
fire-control solving (quadrant elevation / time-of-flight / charge selection), point-mass
through genuine 6-DoF flight dynamics, aerodynamics (spin-stabilized MPMM, base bleed,
Missile DATCOM tables, body-axis missile aero), guidance/navigation/control (GNC) including
GNSS/IMU midcourse and semi-active-laser terminal homing, fragmentation, and live or
post-flight visualization.

The project models real weapon systems (e.g. 155mm M107 and MOD274 base-bleed artillery,
25/35/76mm guns, APKWS-style laser-guided rockets, M31 GMLRS, fin-stabilized darts) as a
way to exercise the framework end-to-end, but the classes are designed to be reused for
arbitrary new munitions, platforms, and engagement scenarios.

> Physical constants and aero coefficients used in the built-in presets are illustrative
> (typical order-of-magnitude values), not validated test data.

## Architecture

All source lives flat in the repository root. The framework is built around a set of
`Base*` abstract classes, each with one or more concrete implementations selected per
scenario:

| Concern | Base class | Implementations |
|---|---|---|
| Flight dynamics | `BaseDynamics` | `Dynamics3Dof`, `Dynamics3DofSpin`, `DynamicsMPMM`, `Dynamics6DoF`, `Dynamics7DoF` (interface stub) |
| Aerodynamics | `BaseAero` | `Aero3DoF_Longitudinal`, `AeroMPMM`, `AeroDatcom`, `AeroMissile6DoF`, `AeroParticle` |
| Propulsion | `BasePropulsion` | `Propulsion_SolidMotor` |
| Guidance | `BaseGuidance` | `ProportionalNav3D`, `GMLRSMidcourseGuidance`, `OptimalAngleFOVGuidance`, `PassthroughGuidance` |
| Control | `BaseController` | `Autopilot6DoF`, `CourseCorrectionController`, `LongitudinalFinController` |
| Sensors | `BaseSensor` | `Imu`, `GnssSensor`, `SALSeeker`, `Seeker3D`, `SeekerLOAL`, `MuzzleVelocityRadar`, `DataLinkReceiver` |
| Targets | `TargetBase` | `TargetStatic`, `TargetConstantVelocity` |
| Integration | `BaseIntegrator` | `RK4FixedStep`, `RK4AdaptiveNearEncounter` |
| Fragmentation | `BaseParticleModel` | `ForwardExplosionParticleModel` |
| Visualization | `BaseVisualizer` | `LiveEngagementPlotter`, `ParticleSwarmPlotter` |
| External models | `BaseExternalModel` | `ExternalModelExe` (shells out to a command-line tool), `ExternalModelSimulink` |

### Model fidelity

- **Point-mass (3-DoF):** `Dynamics3Dof` / `Dynamics3DofSpin` with `Aero3DoF_Longitudinal` —
  pitch follows the flight-path angle as a first-order lag; the spin variant adds a spin-rate state.
- **Modified Point-Mass Model (MPMM):** `DynamicsMPMM` + `AeroMPMM`, a STANAG 4355-style model
  with a yaw-of-repose vector, Magnus effects, latitude-corrected gravity and optional Coriolis.
  It is used by the artillery and gun presets, and includes an airbrake (`airbrake_ADT`) for
  course-correction fuzes.
- **DATCOM aero:** `AeroDatcom` reads Missile DATCOM's CD/CL alpha-by-Mach tables directly
  (via `DatcomReader`).
- **Genuine 6-DoF:** `Dynamics6DoF` + `AeroMissile6DoF` — real attitude, body rates and
  rotational equations of motion, with alpha/beta from the true attitude. Used by the mini
  missile and GMLRS presets.
- **Base bleed:** `Propulsion_DYU` is a drag-reduction helper attached to an aero model's
  `BaseBleed` property (not a `BasePropulsion`).
- **Solid rocket motor:** `Propulsion_SolidMotor` accepts either an average thrust plus burn
  time, or a time-vs-thrust table.

### Simulation loop

Everything is wired together by [`Simulation.m`](Simulation.m), which steps the target,
designator, ground station, and munition GNC/dynamics in lockstep, logs telemetry each
step, and stops on the first tripped termination criterion (impact, altitude bounds,
max downrange, or max time). A [`Munition`](Munition.m) bundles one dynamics, aero,
propulsion, sensor, guidance, and controller instance; guidance and control communicate
through `sensor_bus` / `guid_bus` structs.

Supporting scene objects:
- [`Environment.m`](Environment.m) — ISA atmosphere (0–20 km) and wind, or a measured
  METCM profile via `Environment.loadMetcm` (e.g. `data/metcm_std.txt`).
- [`CoordinateFrame.m`](CoordinateFrame.m) — conversion between the three supported
  position/velocity conventions: `Z-Up` (default), `NED`, and `FDR` (required by
  `DynamicsMPMM`/`AeroMPMM`). A runner sets `sim.Frame` once and `Simulation` propagates it.
- [`LaserDesignator.m`](LaserDesignator.m) — illuminates a target for `SALSeeker` (PRF-code matched).
- [`Jammer.m`](Jammer.m) — GNSS denial zone that `GnssSensor` loses lock inside.
- [`GroundStation.m`](GroundStation.m) + [`OffboardRadar.m`](OffboardRadar.m) — off-board
  tracking that feeds `DataLinkReceiver`.
- [`WeaponPlatform.m`](WeaponPlatform.m), [`ChargeZone.m`](ChargeZone.m) — launchers and
  propellant charge zones.

### Fire control

[`FireControlComputer.m`](FireControlComputer.m) solves for the launch elevation/azimuth
and predicted time-of-flight needed to hit a given target (numerical shooting solver over
the actual aero/propulsion/gravity model). It also handles automatic charge selection
(`solveNumericalShootingAutoCharge`), fuze burst time for a given standoff
(`solveBurstTimeForStandoff`), and airbrake deploy time for course-correction fuzes
(`solveDeployTimeForRange`). Most runner scripts use it to set
up the initial launch state before handing off to `Simulation`.
[`FireMission.m`](FireMission.m) fires multi-round bursts from a `WeaponPlatform` at a
moving target, with realistic inter-round timing and a fresh lead solve for each round.

### Fragmentation

A `Munition` can optionally carry a `BurstTime` and a `ParticleModel`
([`BaseParticleModel.m`](BaseParticleModel.m)): once `Simulation.run()` reaches
`BurstTime`, the single-body flight stops and `Munition.spawnParticles`
releases a swarm of `Particle`s (`Particle < Munition`, flying under
`Dynamics3Dof` + the drag-only `AeroParticle`, no lift/GNC) - each fragment
inherits the parent's forward velocity plus a spin-induced side velocity
(`v = v_cg + spin_rate x r`), per
[`ForwardExplosionParticleModel.m`](ForwardExplosionParticleModel.m).
[`ParticleModelFactory.m`](ParticleModelFactory.m) provides ready-made
cylindrical fragment layouts for 25mm/35mm/76mm rounds.
[`ParticleSwarmSimulation.m`](ParticleSwarmSimulation.m) then runs every
fragment lockstep (each with its own drag coefficient and reference area,
selectable as the shape's minimum or maximum presented cross-section via
`ReferenceAreaMode`) until it reaches the target or loses its chance, shown
live in a second, zoomed-in plot
([`ParticleSwarmPlotter.m`](ParticleSwarmPlotter.m)).

### Factories and data files

Complete weapon systems are assembled via factories:
- [`MunitionFactory.m`](MunitionFactory.m) — builds a projectile/rocket (dynamics + aero +
  propulsion + charge + GNC stack): `createM107Artillery`, `createMOD274`, `create25mmM793`,
  `create35mmHE`, `create76mmHE`, `createHydra70`, `createAPKWS`, `createGuidedRocketDatcom`,
  `createFinStabilizedDart`, `createMiniMissile6DoF`, `createM31GMLRSUnitary`.
- [`WeaponPlatformFactory.m`](WeaponPlatformFactory.m) — builds the launching platform
  (towed 155mm howitzer, Fırtına SPG, 35mm AA mount, 76mm naval mount).
- [`ChargeFactory.m`](ChargeFactory.m) — propellant charge zones.
- [`ParticleModelFactory.m`](ParticleModelFactory.m) — fragment layouts.

Presets can be overridden without touching code via `.mat` files in [`data/`](data):
- [`MunitionData.m`](MunitionData.m) — the standardized per-munition file, named exactly
  after `Munition.Name` (e.g. `data/25mm_M793.mat`, `data/M107_155mm.mat`), holding aero
  coefficient tables, mass/inertia, and charge-dependent form factors.
- [`FactoryData.m`](FactoryData.m) — generic `data/<name>.mat` override for any other field
  a factory method reads; if no file exists, the hardcoded defaults are used.

`generate_example_munition_data.m` and `generate_M107_example_data.m` regenerate the example
data files.

## Scenario builder GUI

[`ScenarioBuilderApp.m`](ScenarioBuilderApp.m) is a control panel: pick a munition, launcher
position and firing mode (Direct — fixed elevation/azimuth/muzzle velocity, or Indirect —
`FireControlComputer` solves onto a target point), then press Run.
[`generateScenarioRunner.m`](generateScenarioRunner.m) writes a standalone, editable runner
script into `generated/` and executes it. The selectable presets come from
[`munitionCatalog.m`](munitionCatalog.m) (currently unguided rounds only: 25mm, 35mm, 76mm,
M107, MOD274).

```matlab
ScenarioBuilderApp()
```

## Example scenarios

Each `runner_*.m` script is a self-contained scenario: build an `Environment`, a `Target`,
a munition (via `MunitionFactory`), solve or specify the launch conditions, run the
`Simulation`, and plot the result.

**Artillery (155mm)**

| Script | Scenario |
|---|---|
| [`runner_m107.m`](runner_m107.m) | M107 HE indirect fire, MPMM model |
| [`runner_m107_direct.m`](runner_m107_direct.m) | M107 direct fire with user-specified launch conditions |
| [`runner_m107_metcm.m`](runner_m107_metcm.m) | M107 direct fire under METCM atmospheric data |
| [`runner_m107_autocharge.m`](runner_m107_autocharge.m) | M107 indirect fire with automatic charge-zone selection |
| [`runner_m107_airbrake_fixed.m`](runner_m107_airbrake_fixed.m) | M107 with a manually set airbrake deploy time |
| [`runner_m107_course_correction.m`](runner_m107_course_correction.m) | M107 closed-loop course-correction airbrake fuze |
| [`runner_mod274.m`](runner_mod274.m) | MOD274 base-bleed HE, extended range via `FireControlComputer` |
| [`runner_mod274_direct.m`](runner_mod274_direct.m) | MOD274 direct fire with user-specified launch conditions |
| [`runner_mod274_datatest.m`](runner_mod274_datatest.m) | Test of the `MunitionData` `.mat` override for MOD274 |

**Guns (25/35/76mm)**

| Script | Scenario |
|---|---|
| [`runner_25mm.m`](runner_25mm.m) | 25mm M793 ballistic launch at a fixed angle |
| [`runner_25mm_fragmentation.m`](runner_25mm_fragmentation.m) | 25mm fragmentation burst vs. anti-drone target |
| [`runner_25mm_velocity_correction.m`](runner_25mm_velocity_correction.m) | 25mm smart-fuze burst-time correction from measured muzzle velocity |
| [`runner_35mm_antidrone.m`](runner_35mm_antidrone.m) | 35mm direct-fire engagement vs. a drone |
| [`runner_35mm_burst.m`](runner_35mm_burst.m) | 35mm multi-round burst with realistic timing |
| [`runner_35mm_fragmentation.m`](runner_35mm_fragmentation.m) | 35mm HE fragmentation burst vs. anti-drone target |
| [`runner_35mm_velocity_correction.m`](runner_35mm_velocity_correction.m) | 35mm smart-fuze burst-time correction |
| [`runner_76mm.m`](runner_76mm.m) | 76mm HE land-to-air engagement, Mach-dependent aero table |
| [`runner_76mm_velocity_correction.m`](runner_76mm_velocity_correction.m) | 76mm smart-fuze burst-time correction |

**Rockets and missiles**

| Script | Scenario |
|---|---|
| [`runner_sam_engagement.m`](runner_sam_engagement.m) | APKWS-class guided interceptor vs. inbound target |
| [`runner_guided_rocket_datcom.m`](runner_guided_rocket_datcom.m) | Guided rocket with Missile DATCOM aero tables |
| [`runner_minimissile_6dof_freeflight.m`](runner_minimissile_6dof_freeflight.m) | Mini missile 6-DoF free-flight physics check |
| [`runner_minimissile_6dof_intercept.m`](runner_minimissile_6dof_intercept.m) | Mini missile 6-DoF guided intercept with live plotting |
| [`runner_m31_gmlrs_direct.m`](runner_m31_gmlrs_direct.m) | M31 GMLRS 6-DoF flight only, no guidance |
| [`runner_m31_gmlrs_himars.m`](runner_m31_gmlrs_himars.m) | M31 GMLRS from HIMARS: GNSS/IMU midcourse + SAL terminal vs. moving target |

**Other**

| Script | Scenario |
|---|---|
| [`runner_fin_stabilized_dart.m`](runner_fin_stabilized_dart.m) | Fin-stabilized dart toss, plain 3-DoF longitudinal model |

Run any of them directly in MATLAB from the repo root, e.g.:

```matlab
runner_mod274
```

## Requirements

MATLAB. The core framework and the scenario builder GUI need no toolboxes beyond base
MATLAB. `ExternalModelSimulink` requires Simulink.

## Status

Active development — expect interfaces on the `Base*` classes to evolve as new dynamics
models, aero tables, and guidance laws are added.
