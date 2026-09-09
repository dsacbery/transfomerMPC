# Day 3 Completion Handoff

## Current Status

Day 3 creates a reproducible, mock-only Simulink top-level skeleton. The
model is `model/tmpsim_online.slx`, the existing frozen Day 2 dictionary is
`model/tmpsim_online.sldd`, and the model can compile and run a 0.2-second
minimal simulation without CarSim.

## Completed And Verified

- `scripts/create_tmpsim_online_model.m` deterministically rebuilds and saves
  `model/tmpsim_online.slx` without modifying the Day 2 dictionary.
- The model attaches `tmpsim_online.sldd` using Simulink's model-relative
  dictionary reference, which is the R2024b-supported form for this model.
- The top level contains exactly these Day 3 subsystems:
  `ScenarioManager`, `CarSimAdapter`, `ErrorFeatureBuilder`, `HistoryBuffer`,
  `TransformerRisk`, `RiskSupervisor`, `LateralControllerVariant`,
  `SpeedOuterLoop`, and `Logger`.
- All subsystem Bus ports use the Day 2 frozen types: `RefBus`, `MeasBus`,
  `ErrBus`, `HistoryBus`, `RiskRawBus`, `RiskBus`, `MPCParamBus`, and `CmdBus`.
- Mock paths use only standard Simulink blocks: Constant, Bus Creator, Inport,
  Outport, Unit Delay, and Terminator. Bus Creator input lines are named after
  their frozen Bus elements, so model compilation has no element-name warnings.
- The fixed-step model rate is `Ts_mpc=0.02 s`; the TransformerRisk mock
  output and placeholder delay use `Ts_tr=0.10 s`. Timing is assigned to
  blocks/ports. Day 2 `Simulink.BusElement` objects retain their default
  inherited sample time (`-1`) and are not assigned a discrete sample time.
- `tests/TestOnlineModelSkeleton.m` has six passing tests: dictionary binding,
  dictionary preservation, required top-level subsystems, compile/rate checks,
  BusElement timing ownership, and a 0.2-second mock simulation.

## Verification Commands

Run from the project root:

```matlab
addpath('scripts'); addpath('config'); addpath('src');

cfg = tmpsim_config();
tmpsim.validateOnlineConfig(cfg);
create_tmpsim_online_model();

results = runtests({'tests/TestSignalContracts.m', ...
    'tests/TestOnlineModelSkeleton.m'});
assertSuccess(results)

load_system('model/tmpsim_online.slx')
set_param('tmpsim_online', 'SimulationCommand', 'update')
out = sim('model/tmpsim_online.slx', 'StopTime', '0.20');
assert(out.SimulationMetadata.ModelInfo.StopTime >= 0.20)
close_system('tmpsim_online', 0)
```

Observed environment: MATLAB/Simulink R2024b. The Day 3 test suite passed
6/6 after a clean model rebuild; the Day 2 contract suite was also rerun at
the start of Day 3 and passed.

## Timing And Interface Boundaries

- Day 3 mock model fixed step and later online output logging step:
  `0.02 s`.
- Transformer mock rate: `0.10 s`.
- CarSim's actual mathematical integration step remains `0.0005 s`; it is a
  future external-plant setting and is intentionally not modeled in this
  Day 3 mock-only Simulink skeleton.
- No CarSim S-Function, CarSim installation content, generated log, cache, or
  license file is in the repository.

## Explicitly Deferred

- `src/+tmpsim/CarSimAdapter.m` and all raw CarSim signal mapping/units/sign
  conversion are Day 4 work.
- `ErrorFeatureBuilder`, `HistoryBuffer`, Transformer inference, Risk
  Supervisor, lateral MPC, and speed-control algorithms are not implemented.
- The `CarSimAdapter` subsystem only provides typed finite mock `MeasBus`
  output and terminates `CmdBus`; it has no external plant connection.

## Start Day 4 Next

1. Verify the CarSim 2019.1 and MATLAB R2024b Simulink interface on the
   local machine before adding a plant block.
2. Implement `src/+tmpsim/CarSimAdapter.m` only after that compatibility
   check, using `docs/CarSim_2019_1_Signal_Inventory.md` for the frozen
   raw-to-SI mapping and left-positive convention.
3. Retain the Day 3 Bus names, dimensions, types, and model rate. Do not
   introduce Transformer, Risk Supervisor, MPC, or speed-control algorithms
   until their scheduled development days.

## Git State

Day 3 work is committed locally on `codex/day3-online-skeleton`. No remote
push was performed.
