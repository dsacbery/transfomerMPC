# Day 4 CarSim Adapter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement a testable CarSim measurement and command boundary that preserves the frozen Day 2 buses and the Day 3 model skeleton.

**Architecture:** `tmpsim.CarSimAdapter` owns only two stateful transformations: named CarSim raw measurements to the frozen `MeasBus` shape, and a `CmdBus`-shaped command to explicit CarSim import channels. The measured steering derivative is sampled at `Ts_mpc=0.02 s`; invalid measurements emit finite safe values and reset derivative history. Command scaling is configured explicitly because the confirmed `IMP_PCON_BK` interface uses MPa while `IMP_THROTTLE_ENGINE` is dimensionless.

**Tech Stack:** MATLAB R2024b, `matlab.unittest`, Simulink, CarSim 2019.1.

---

### Task 1: Lock Down Existing Contracts

**Files:**
- Test: `tests/TestSignalContracts.m`
- Test: `tests/TestOnlineModelSkeleton.m`

- [x] Run `TestSignalContracts` and record its result.
- [x] Run `TestOnlineModelSkeleton` and record its result.
- [x] Restore `model/tmpsim_online.slx` when its deterministic test builder changes the tracked binary.

### Task 2: Measurement Adapter Red-Green Cycle

**Files:**
- Create: `tests/TestCarSimAdapter.m`
- Create: `src/+tmpsim/CarSimAdapter.m`

- [x] Write a failing test for the named raw measurement interface:

```matlab
adapter = tmpsim.CarSimAdapter(0.02, commandMap);
meas = adapter.adaptMeasurement(raw);
testCase.verifyEqual(meas.yaw, deg2rad(raw.Yaw), AbsTol=1e-12);
testCase.verifyEqual(meas.vx, raw.Vx / 3.6, AbsTol=1e-12);
testCase.verifyEqual(meas.delta_meas, ...
    deg2rad((raw.Steer_L1 + raw.Steer_R1) / 2), AbsTol=1e-12);
```

- [x] Run `runtests('tests/TestCarSimAdapter.m')` and confirm it fails because `tmpsim.CarSimAdapter` does not exist.
- [x] Implement only the raw conversions for `Xo`, `Yo`, `Yaw`, `Vx`, `Vy`, `AVz`, `Beta`, `Ay`, `Steer_L1`, `Steer_R1`, and `Ax`.
- [x] Re-run the focused test and confirm it passes.
- [x] Add a second failing test for the first sample and second-sample steering difference:

```matlab
adapter.adaptMeasurement(rawAtZero);
meas = adapter.adaptMeasurement(rawAtOneTick);
testCase.verifyEqual(meas.delta_rate_meas, ...
    (meas.delta_meas - previousDelta) / 0.02, AbsTol=1e-12);
```

- [x] Implement the minimum derivative state and re-run the focused test.
- [x] Add an invalid-input test that verifies `meas_valid=false`, finite zero outputs, and derivative-history reset; implement only the corresponding fail-safe path.

### Task 3: Command Boundary Red-Green Cycle

**Files:**
- Modify: `tests/TestCarSimAdapter.m`
- Modify: `src/+tmpsim/CarSimAdapter.m`

- [x] Write a failing test that a finite valid command maps left-positive `delta_cmd` to equal positive `IMP_STEER_L1` and `IMP_STEER_R1` values in degrees.
- [x] Write a failing test that positive `ax_cmd` produces nonzero `IMP_THROTTLE_ENGINE` and zero `IMP_PCON_BK`, while negative `ax_cmd` reverses that relation.
- [x] Run the focused tests and confirm the command method is absent.
- [x] Implement configured amplitude limits, finite-value rejection, command holding, and mutually exclusive throttle/brake allocation.
- [x] Re-run the focused adapter suite and confirm it passes.

### Task 4: Interface and Smoke Evidence

**Files:**
- Create: `tests/TestCarSimStraightSmoke.m`
- Modify: `docs/Day_04_Handoff.md`

- [x] Add a 10-second straight-log assertion helper for monotonic `x`, positive `vx`, bounded yaw increments, zero-steer behavior, and throttle/brake mutual exclusion.
- [ ] Run it against the available CarSim output only after the active CarSim configuration is confirmed to expose the required command ports; do not embed local paths in source. Blocked by the confirmed `AAy`/`Ay` and longitudinal-port mismatch.
- [x] Record the exact runtime command, observed CarSim loading result, port/units evidence, and any integration limitation in the Day 4 handoff.

### Task 5: Final Regression and Commit

**Files:**
- Modify: `src/+tmpsim/CarSimAdapter.m`
- Modify: `tests/TestCarSimAdapter.m`
- Create: `tests/TestCarSimStraightSmoke.m`
- Create: `docs/Day_04_Handoff.md`

- [x] Run `TestSignalContracts`, `TestOnlineModelSkeleton`, `TestCarSimAdapter`, and the smoke test.
- [ ] Run `git diff --check` and inspect for paths, generated artifacts, and dictionary/model changes.
- [ ] Commit only Day 4 source, tests, plan, and handoff documentation with `feat: add CarSim adapter boundary`.
