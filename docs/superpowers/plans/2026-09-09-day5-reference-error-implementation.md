# Day 5 Reference And Error Feature Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `subagent-driven-development` (recommended) or `executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a pure-MATLAB reference-to-error feature path that preserves the frozen RefBus, MeasBus, ErrBus, and 14-column feature contracts.

**Architecture:** `tmpsim.ReferenceManager` owns only the monotonic path index and returns a scalar RefBus-style structure from a caller-supplied `N x 5` path. `tmpsim.ErrorFeatureBuilder` owns only the previous lateral error used at low speed, derives a scalar ErrBus-style structure from RefBus and MeasBus, and creates the frozen feature row without touching the Simulink model or data dictionary.

**Tech Stack:** MATLAB R2024b, `handle` classes, `matlab.unittest`, existing `tmpsim_config` frozen contract.

---

### Task 1: ReferenceManager Contract

**Files:**
- Create: `tests/TestReferenceManager.m`
- Create: `src/+tmpsim/ReferenceManager.m`

- [ ] **Step 1: Write failing reference-path tests**

```matlab
manager = tmpsim.ReferenceManager(path, 10.0, 1.0);
first = manager.step(meas(2.1, 0.0), false);
later = manager.step(meas(1.1, 0.0), false);
testCase.verifyEqual(later.path_idx, first.path_idx);
```

Cover forward-only nearest-point lookup, terminal-point locking with `ref_valid=false`, reset to path index one, explicit `[v_min, scenario_speed_limit]` speed clipping, and straight/circular/lane-change test paths. The lane-change fixture uses the supplied analytic `tanh` trajectory only inside the test.

- [ ] **Step 2: Run tests to verify RED**

Run:

```powershell
matlab -batch "addpath('src'); results = runtests('tests/TestReferenceManager.m'); assertSuccess(results)"
```

Expected: failures caused by the missing `tmpsim.ReferenceManager` class.

- [ ] **Step 3: Write minimal ReferenceManager implementation**

```matlab
classdef ReferenceManager < handle
    methods
        function obj = ReferenceManager(pathXYpsiKappaV, speedLimitMps, vMinMps)
        end
        function ref = step(obj, meas, reset)
        end
        function reset(obj)
        end
    end
end
```

Validate finite `N x 5` path data and explicit finite limits, search only from the held `path_idx` through the forward window, clamp at the final row, and return the seven frozen RefBus fields with `path_idx` as `uint32`.

- [ ] **Step 4: Run reference tests to verify GREEN**

Run the Step 2 command.

Expected: all `TestReferenceManager` tests pass.

### Task 2: ErrorFeatureBuilder Contract

**Files:**
- Create: `tests/TestErrorFeatureBuilder.m`
- Create: `src/+tmpsim/ErrorFeatureBuilder.m`

- [ ] **Step 1: Write failing error and feature tests**

```matlab
builder = tmpsim.ErrorFeatureBuilder(0.02);
[err, feature] = builder.build(ref, meas, false);
testCase.verifyEqual(feature, [meas.vx, meas.vy, ... ref.v_ref_base]);
testCase.verifySize(feature, [1 14]);
```

Cover straight-path zero error and exact feature order, Frenet left-positive error, `+-pi` heading wrapping, constant-curvature error rates, low-speed finite-difference fallback with `err_valid=false`, reset clearing the derivative state, invalid input containment, and a ReferenceManager-to-ErrorFeatureBuilder lane-change data-path test.

- [ ] **Step 2: Run tests to verify RED**

Run:

```powershell
matlab -batch "addpath('src'); results = runtests('tests/TestErrorFeatureBuilder.m'); assertSuccess(results)"
```

Expected: failures caused by the missing `tmpsim.ErrorFeatureBuilder` class.

- [ ] **Step 3: Write minimal ErrorFeatureBuilder implementation**

```matlab
classdef ErrorFeatureBuilder < handle
    methods
        function obj = ErrorFeatureBuilder(tsMpc)
        end
        function [err, latestFeature] = build(obj, ref, meas, reset)
        end
        function reset(obj)
        end
    end
end
```

Compute `e_y`, wrapped `e_psi`, geometric rates above `0.5 m/s`, and clipped finite-difference `e_y_rate` below that threshold. Assemble columns in the frozen 14-field order and return finite `double` output under every input condition.

- [ ] **Step 4: Run error tests to verify GREEN**

Run the Step 2 command.

Expected: all `TestErrorFeatureBuilder` tests pass.

### Task 3: Regression And Handoff

**Files:**
- Create: `docs/Day_05_Handoff.md`

- [ ] **Step 1: Run complete regression**

```matlab
addpath('scripts'); addpath('config'); addpath('src');
results = runtests({'tests/TestSignalContracts.m', ...
    'tests/TestOnlineModelSkeleton.m', 'tests/TestCarSimAdapter.m', ...
    'tests/TestCarSimStraightSmoke.m', 'tests/TestReferenceManager.m', ...
    'tests/TestErrorFeatureBuilder.m'});
assertSuccess(results)
```

Restore only `model/tmpsim_online.slx` if the existing Day 3 model test regenerates it.

- [ ] **Step 2: Record the handoff**

Document the exact verified commands/results, constructor and method contracts, fixed feature order, and the unresolved Day 4 real CarSim straight-run limitation. Identify Day 6 as the start of `HistoryBuffer`; do not implement it now.

- [ ] **Step 3: Inspect the commit boundary**

```powershell
git diff --check
git status --short
rg -n -i '([A-Za-z]:\\|carsim|slprj|\.slxc|license|\.mat$|\.csv$)' --glob '!docs/Day_04_Handoff.md' --glob '!docs/Day_05_Handoff.md'
```

Review the output and stage only Day 5 source, tests, plan, and handoff files. Do not stage `.sldd`, regenerated `.slx`, CarSim content, generated caches, license material, or result artifacts.

- [ ] **Step 4: Commit locally**

```powershell
git add docs/Day_05_Handoff.md docs/superpowers/plans/2026-09-09-day5-reference-error-implementation.md tests/TestReferenceManager.m tests/TestErrorFeatureBuilder.m src/+tmpsim/ReferenceManager.m src/+tmpsim/ErrorFeatureBuilder.m
git commit -m "feat: add reference error feature pipeline"
```

Do not push or merge.
