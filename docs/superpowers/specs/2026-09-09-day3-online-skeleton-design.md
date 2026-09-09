# Day 3 Online Skeleton Design

## Scope

Create a reproducible Simulink top-level skeleton at `model/tmpsim_online.slx` and attach `model/tmpsim_online.sldd`. The model is a compileable/minimal-simulation scaffold only. It must not implement CarSim signal conversion, Transformer inference, risk supervision, MPC, or speed-control algorithms.

## Architecture

The build script `scripts/create_tmpsim_online_model.m` will create the model from standard Simulink blocks and save it deterministically. The top level contains these named subsystems:

`ScenarioManager`, `CarSimAdapter`, `ErrorFeatureBuilder`, `HistoryBuffer`, `TransformerRisk`, `RiskSupervisor`, `LateralControllerVariant`, `SpeedOuterLoop`, and `Logger`.

The model uses two explicit rates:

- `Ts_mpc = 0.02 s` for the top-level control skeleton and all high-rate mock paths.
- `Ts_tr = 0.10 s` for the TransformerRisk mock path, implemented with a block sample time rather than a `Simulink.BusElement` sample time.

The model configuration uses a fixed-step solver with step size `Ts_mpc`, a finite short stop time, and the Day 2 data dictionary. No CarSim S-Function or external plant is included; `CarSimAdapter` is a mock measurement/command boundary for Day 3.

## Bus Contracts And Data Flow

Every typed interface uses only the Day 2 buses in `model/tmpsim_online.sldd`: `RefBus`, `MeasBus`, `ErrBus`, `HistoryBus`, `RiskRawBus`, `RiskBus`, `MPCParamBus`, and `CmdBus`.

The scaffold creates valid default instances for each bus with the exact frozen field names, types, and dimensions. Bus Creator blocks assemble those values, and Bus Selector blocks expose only the fields needed to complete the mock signal path. Unit Delay blocks provide deterministic stateful placeholders for history/risk/command retention. Unused algorithmic outputs are bounded finite constants with valid status flags.

To keep the model compileable without implementing Day 4+, the top-level wiring is a closed mock loop: ScenarioManager provides `RefBus`; CarSimAdapter provides `MeasBus`; ErrorFeatureBuilder produces `ErrBus` and a 1x14 feature vector; HistoryBuffer produces `HistoryBus`; TransformerRisk produces `RiskRawBus`; RiskSupervisor produces `RiskBus`, `MPCParamBus`, and adapted speed; LateralControllerVariant and SpeedOuterLoop produce command components; Logger consumes the buses/signals. CarSimAdapter's command-side mock terminates `CmdBus` without external I/O.

## Verification

Add `tests/TestOnlineModelSkeleton.m` with test-first checks for:

1. The build script creates the model and associates the expected data dictionary.
2. All nine required subsystem names exist at the top level.
3. The model compiles/update-diagram without bus type errors, sample-time conflicts, or unconnected ports.
4. A short normal simulation completes and produces finite mock outputs.
5. The dictionary's `Simulink.BusElement` objects have no sample-time assignments; timing is owned by blocks/ports.

The test will first be run against the missing model/build script to establish the expected failing state, then the minimum implementation will be added and the full Day 2 plus Day 3 verification suite rerun. Generated Simulink cache/build artifacts remain ignored and are not committed.

## Handoff

Update `docs/Day_03_Handoff.md` with the exact model/build/test commands, observed MATLAB/Simulink release, verification results, known limitation that CarSim-Simulink compatibility remains a Day 3/4 interface check, and the commit identifier. Do not push to any remote in this task.
