# Day 4 Completion Handoff

## Current Status

Day 4 adds a testable CarSim measurement and command boundary without
changing the frozen Day 2 data dictionary or the Day 3 top-level model
skeleton. `tmpsim.CarSimAdapter` is currently a MATLAB boundary object; it
does not yet replace the mock-only `CarSimAdapter` subsystem in
`tmpsim_online.slx`.

## Completed And Verified

- `src/+tmpsim/CarSimAdapter.m` maps named raw measurements to exactly the
  frozen `MeasBus` field set:
  `x`, `y`, `yaw`, `vx`, `vy`, `yaw_rate`, `beta`, `ay`, `delta_meas`,
  `ax_meas`, `delta_rate_meas`, and `meas_valid`.
- The adapter applies the recorded conversions: `Yaw`, `AVz`, and `Beta`
  from deg(/s) to rad(/s); `Vx` and `Vy` from km/h to m/s; `Ay` and `Ax`
  from g to m/s^2; and the mean of `Steer_L1` and `Steer_R1` from deg to
  rad. It preserves the established left-positive convention.
- `delta_rate_meas` is the discrete difference of valid `delta_meas` samples
  at the enforced frozen `Ts_mpc=0.02 s`. The first valid sample and the
  first sample after an invalid measurement report zero rate.
- A nonfinite or incomplete raw signal set produces finite zero measurement
  values with `meas_valid=false`; it cannot propagate `NaN` or `Inf`.
- `reset` clears the steering-difference history and the held plant command,
  so a simulation restart begins with zero measured steering rate and zero
  held steering/throttle/brake output.
- The command boundary maps `delta_cmd` to equal left/right front-road-wheel
  imports in deg, and maps positive/negative `ax_cmd` to mutually exclusive
  `IMP_THROTTLE_ENGINE` and `IMP_PCON_BK` outputs. Amplitude limits and the
  `cmd_valid`/`hold_last_cmd` policy are enforced before the output is saved
  as the next hold value.
- The command map is intentionally explicit. `IMP_THROTTLE_ENGINE` is
  dimensionless while `IMP_PCON_BK` is MPa, so the brake pressure limit is a
  required caller-supplied calibration, not a guessed repository constant.
- `src/+tmpsim/verifyCarSimStraightSmoke.m` implements the 10-second,
  `0.02 s` acceptance checks: timing, monotonic forward `x`, positive `vx`,
  bounded per-sample yaw change, zero `delta_cmd`, and throttle/brake
  mutual exclusion.

## CarSim Interface Findings

- MATLAB R2024b resolved the 64-bit CarSim `vs_sf.mexw64` and `vs_sf2v.mexw64`
  S-Functions. A temporary copy of the active simfile was changed to use the
  64-bit vehicle DLL and a temporary output directory; the CarSim base model
  loaded the DLL and completed a `0.02 s` simulation. CarSim reported
  `Call Terminate at t = 0.020000`.
- The active Day 2 evidence run records `EXT_MODEL_STEP=0.00050000`; this is
  the CarSim mathematical integration step. Day 4 adapter and online logging
  remain at `Ts_mpc=0.02 s`.
- Current metadata confirms `Ay` is instantaneous-CG lateral acceleration in
  `g`. It also confirms `AAy` is body-fixed pitch angular acceleration in
  `rad/s^2`. The currently selected `Trans_01` export configuration contains
  `AAy`, not `Ay`, so it is incompatible with the frozen `MeasBus.ay`
  contract. The adapter rejects an `AAy`-only input rather than silently using
  the wrong physical quantity.
- Current CarSim import tables confirm `IMP_STEER_L1` and `IMP_STEER_R1` use
  deg road-wheel angles, `IMP_THROTTLE_ENGINE` is dimensionless, and
  `IMP_PCON_BK` is brake-master-cylinder pressure in MPa.

## Verification

Run from the repository root:

```matlab
addpath('scripts'); addpath('config'); addpath('src');
results = runtests({'tests/TestSignalContracts.m', ...
    'tests/TestOnlineModelSkeleton.m', ...
    'tests/TestCarSimAdapter.m', ...
    'tests/TestCarSimStraightSmoke.m'});
assertSuccess(results)
```

Observed in MATLAB R2024b:

- `TestSignalContracts`: 11 passed.
- `TestOnlineModelSkeleton`: 6 passed.
- `TestCarSimAdapter`: 9 passed.
- `TestCarSimStraightSmoke`: 2 passed.
- Total: 28 passed, 0 failed.

The Day 3 builder rewrites `model/tmpsim_online.slx` during its own test; that
generated binary was restored afterward. `model/tmpsim_online.sldd` was not
edited or regenerated.

The short real-interface check used the local CarSim base model with a
temporary simfile and temporary output directory. It validated S-Function and
solver loading plus a `0.02 s` run. Its final no-save close attempt warned
because the temporary mask value marked the in-memory model dirty; no model
was saved and the simulated run had already completed.

## Known Limitations

- A real 10-second C-Class closed-plant straight run is not certified yet.
  The currently active Day 2 run is a DLC case with no Simulink ports. The
  available `Trans_01` Simulink configuration exports `AAy` instead of `Ay`
  and imports `IMP_SPEED` plus wheel angles rather than the required
  throttle/brake pair.
- A C-Class mapping from requested longitudinal acceleration to the physical
  maximum brake-master-cylinder pressure has not been validated. The value
  `10 MPa` in `TestCarSimAdapter` is a test fixture only, not a vehicle
  calibration.
- The 10-second test suite currently verifies the acceptance rule using a
  deterministic `0.02 s` log fixture. It must be rerun against a real CarSim
  log once a compatible local CarSim run is generated.

## Start Day 5 Next

1. In the local CarSim database, create a non-repository C-Class straight-run
   configuration that exports `Xo`, `Yo`, `Yaw`, `Vx`, `Vy`, `AVz`, `Beta`,
   `Ay`, `Steer_L1`, `Steer_R1`, and `Ax`, and imports the confirmed steering
   plus throttle/brake channels.
2. Validate the C-Class brake-pressure limit and the acceleration-to-actuator
   calibration, then pass those values explicitly to `CarSimAdapter`.
3. Run the real 10-second straight log at `0.02 s`, feed it to
   `tmpsim.verifyCarSimStraightSmoke`, and record the resulting acceptance
   evidence before connecting the Day 4 adapter to the online model.
4. Preserve the frozen dictionary and all nine Day 3 subsystems while starting
   Day 5 reference and error-feature work.
