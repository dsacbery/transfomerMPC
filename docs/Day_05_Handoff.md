# Day 5 Completion Handoff

## Current Status

Day 5 adds a pure MATLAB reference-to-error feature path without changing the
frozen Day 2 dictionary or the Day 3 top-level Simulink skeleton. The new
classes are not connected to `model/tmpsim_online.slx` yet. They are isolated
from CarSim and can be unit tested using scalar RefBus- and MeasBus-shaped
structures.

## Git Baseline And Branch

- Day 5 was created locally from `dcb907a feat: add CarSim adapter boundary`
  on `codex/day4-carsim-adapter`, not from `develop`.
- Local Day 5 branch: `codex/day5-reference-error`.
- Before branching, `git status --porcelain=v2 --branch` reported
  `branch.upstream origin/codex/day4-carsim-adapter` and `branch.ab +0 -0`.
- `git pull --ff-only origin codex/day4-carsim-adapter` was attempted twice.
  Both attempts failed because the local environment could not connect to
  GitHub (connection reset / port 443 connection failure). The local branch
  and its previously fetched upstream reference both resolved to `dcb907a`.
- This branch has not been pushed or merged.

## Completed And Verified

- `src/+tmpsim/ReferenceManager.m` accepts a caller-supplied finite `N x 5`
  double path in `[x, y, psi, kappa, v]` order plus explicit scenario speed
  limit and minimum speed. It returns exactly the frozen RefBus field set:
  `x_ref`, `y_ref`, `psi_ref`, `kappa_ref`, `v_ref_base`, `path_idx`, and
  `ref_valid`.
- `ReferenceManager.step(meas, reset)` searches only forward from its held
  index, inspecting at most 50 path points. It cannot regress `path_idx`.
  It uses `hypot` for robust nearest-point distances, including large finite
  coordinate values. Invalid, missing, false, or nonlogical `meas_valid`
  holds the current path index.
- Reset gives precedence to returning path index one. The final path point is
  locked, does not index out of range, and has `ref_valid=false`.
- `src/+tmpsim/ErrorFeatureBuilder.m` accepts RefBus- and MeasBus-shaped
  structures and returns exactly the frozen ErrBus field set plus a finite
  `1x14 double` `latestFeature` row.
- The high-speed error equations are:

  ```text
  e_y        = -sin(psi_ref) * (x - x_ref) + cos(psi_ref) * (y - y_ref)
  e_psi      = wrapAngle(yaw - psi_ref)
  e_y_rate   = vy + vx * sin(e_psi)
  e_psi_rate = yaw_rate - vx * kappa_ref
  ```

- `wrapAngle` uses `atan2(sin(angle), cos(angle))`. The positive lateral
  error convention is therefore the frozen left-positive convention.
- For `vx < 0.5 m/s`, `e_y_rate` is the finite difference from the previous
  valid lateral error, limited to `[-5, 5] m/s`; `err_valid=false` for that
  sample. Reset clears this saved lateral-error state.
- Invalid buses return finite zero ErrBus and feature values without updating
  the saved lateral-error state. A valid but terminal `RefBus` still yields
  finite values but has `err_valid=false` through `ref_valid`.
- Frozen feature order is unchanged:

  ```text
  vx, vy, yaw_rate, ay, beta, delta_meas, delta_rate_meas, ax_meas,
  e_y, e_psi, e_y_rate, e_psi_rate, kappa_ref, v_ref_base
  ```

## Tests

The following commands were run from the repository root in MATLAB R2024b.

```powershell
matlab -batch "addpath('scripts'); addpath('config'); addpath('src'); results = runtests({'tests/TestSignalContracts.m', 'tests/TestOnlineModelSkeleton.m', 'tests/TestCarSimAdapter.m', 'tests/TestCarSimStraightSmoke.m'}); assertSuccess(results)"
```

Result: `28 Passed, 0 Failed, 0 Incomplete` in 25.66 seconds. The Day 3
test regenerated `model/tmpsim_online.slx`; that tracked binary was restored
before Day 5 edits.

```powershell
matlab -batch "addpath('src'); results = runtests('tests/TestReferenceManager.m'); assertSuccess(results)"
```

RED result before implementation: `MATLAB:undefinedVarOrClass` for missing
`tmpsim.ReferenceManager`. GREEN result: `12 Passed, 0 Failed, 0 Incomplete`.

```powershell
matlab -batch "addpath('src'); results = runtests('tests/TestErrorFeatureBuilder.m'); assertSuccess(results)"
```

RED result before implementation: `MATLAB:undefinedVarOrClass` for missing
`tmpsim.ErrorFeatureBuilder`. GREEN result: `10 Passed, 0 Failed, 0
Incomplete`.

```powershell
matlab -batch "addpath('scripts'); addpath('config'); addpath('src'); results = runtests({'tests/TestSignalContracts.m', 'tests/TestOnlineModelSkeleton.m', 'tests/TestCarSimAdapter.m', 'tests/TestCarSimStraightSmoke.m', 'tests/TestReferenceManager.m', 'tests/TestErrorFeatureBuilder.m'}); assertSuccess(results)"
```

Final result: `50 Passed, 0 Failed, 0 Incomplete` in 30.40 seconds. The two
existing Day 3 dictionary-name resolution warnings appeared during model
tests but did not fail any test. `model/tmpsim_online.slx` was restored after
this final test run; `model/tmpsim_online.sldd` was never modified.

`checkcode` produced no findings for the two new classes and their test
files.

## Test Coverage

- `TestReferenceManager.m`: straight path, left-turn constant-curvature arc,
  supplied `tanh` lane-change fixture, forward-only lookup, 50-point search
  window, terminal lock, reset, explicit speed bounds, invalid measurement
  hold, frozen fields, and finite large-coordinate behavior.
- `TestErrorFeatureBuilder.m`: straight-path zero error and feature order,
  left-positive Frenet error, heading wrap across `+-pi`, constant-curvature
  rate calculation, low-speed fallback, reset, invalid-bus containment, and
  the `ReferenceManager -> ErrorFeatureBuilder` lane-change data path.

## Known Limitations

- Day 4's real 10-second C-Class closed-plant straight run is still not
  certified. The active CarSim export configuration supplies `AAy`, which is
  pitch angular acceleration, not the required lateral acceleration `Ay`.
  Do not substitute `AAy` for `Ay` or treat the deterministic smoke fixture
  as real-plant acceptance evidence.
- A validated C-Class throttle/brake acceleration calibration and brake
  pressure limit are still absent. The 10 MPa value used in Day 4 tests is a
  fixture, not a vehicle parameter.
- The Day 5 MATLAB classes are deliberately not wired into the Day 3
  `ErrorFeatureBuilder` subsystem. There is no HistoryBuffer, Transformer,
  Risk Supervisor, MPC, or speed outer loop implementation yet.
- `ReferenceManager` requires a caller to provide the reference path and
  speed limits. It does not create a formal vehicle, road, or experimental
  scenario configuration; the lane-change formula is only a unit-test
  fixture.

## Start Day 6 Next

1. Implement `src/+tmpsim/HistoryBuffer.m` and `tests/TestHistoryBuffer.m`
   only, consuming the established finite `1x14 double` feature row.
2. Keep the history contract frozen at a `16x14 double` chronological window,
   zero-filled while not ready, and resettable with no change to feature
   order.
3. Do not add Transformer inference, risk logic, MPC, speed control, or any
   CarSim model connection as part of Day 6.
4. Continue preserving `model/tmpsim_online.sldd` and restore the generated
   `model/tmpsim_online.slx` after any existing model test.
