# Day 2 Completion Handoff

## Current Status

Day 2 is complete. The Simulink Data Dictionary, eight Bus contracts, executable configuration, vehicle parameters, feature order, and configuration validation are implemented and verified.

## Completed and Verified

- MATLAB R2024b has Simulink, Deep Learning Toolbox, and Model Predictive Control Toolbox. `mpcActiveSetSolver` and `dlnetwork` are available.
- CarSim 2019.1 completed the `day2_export_smoke` copy of the Quick Start `DLC @ 120 km/h` case and produced usable plots.
- The internal `MeasBus` mapping and SI conversions are recorded in `docs/CarSim_2019_1_Signal_Inventory.md`.
- `model/tmpsim_online.sldd` contains `RefBus`, `MeasBus`, `ErrBus`, `HistoryBus`, `RiskRawBus`, `RiskBus`, `MPCParamBus`, and `CmdBus`.
- `HistoryBus.window` is `16x14 double`; continuous values are `double`, flags are `boolean`, and status or mode IDs are `uint8`.
- `scripts/create_tmpsim_dictionary.m` recreates the Data Dictionary contract deterministically.
- `config/tmpsim_config.m` freezes P0 sampling, history, MPC, risk, speed, state-code, feature-order, and left-positive sign conventions.
- `config/vehicle_params.m` stores the C-Class Hatchback parameters derived from the evidence run, including nominal small-slip tire linearization values.
- `src/+tmpsim/validateOnlineConfig.m` rejects invalid sample-rate ratios, history shapes, feature orders, bounds, and optional feature-statistics metadata.
- `tests/TestSignalContracts.m` contains eleven passing MATLAB contract tests.

Run the verification from the project root:

```matlab
cd('D:\Desktop\TransformerMPC')
addpath('scripts')
addpath('config')
addpath('src')
create_tmpsim_dictionary()

cfg = tmpsim_config();
tmpsim.validateOnlineConfig(cfg)

results = runtests('tests/TestSignalContracts.m');
assertSuccess(results)
```

## Frozen Feature Order

```text
vx, vy, yaw_rate, ay, beta, delta_meas, delta_rate_meas,
ax_meas, e_y, e_psi, e_y_rate, e_psi_rate, kappa_ref, v_ref_base
```

## Evidence Vehicle Parameters

- C-Class Hatchback from `day2_export_smoke`: `m=1501 kg`, `Iz=2192.089539 kg*m^2`, wheelbase `2.910 m`, `lf=1.049562958 m`, and `lr=1.860437042 m`.
- The nominal small-slip tire-table linearization gives `Cf=159055.11037704 N/rad` and `Cr=92776.4754843415 N/rad`.
- Treat `Cf` and `Cr` as nominal linearization values. Before accepting the Day 7 lateral MPC, verify the operating-point choice against CarSim; do not replace them with guessed values.
- The left-positive convention is verified: `delta > 0`, `yaw > 0`, and `ay > 0` represent a left turn. The raw-to-SI mapping remains in `docs/CarSim_2019_1_Signal_Inventory.md`.

## Start Day 3 Next

1. Create `model/tmpsim_online.slx` and attach `model/tmpsim_online.sldd`.
2. Add only the Day 3 top-level subsystem skeleton and mock outputs; do not implement `CarSimAdapter.m` yet.
3. Set `Ts_mpc=0.02 s` at block or port level. Do not set sample time on `Simulink.BusElement`.
4. Verify CarSim 2019.1 can launch a Simulink interface using MATLAB R2024b before relying on the external plant connection.
5. Do not connect the Transformer, Risk Supervisor, or MPC before the fixed-MPC baseline stage. Do not add CarSim installation files, generated results, caches, or license files to this repository.

## Local CarSim Context

- CarSim program: `D:\Carsim\carsim2019.1`.
- Active local database: `D:\Desktop\Carsim`.
- Evidence run: `day2_export_smoke`, based on Quick Start `DLC @ 120 km/h` and C-Class Hatchback.
- The CarSim 2019.1 and MATLAB R2024b Simulink interface is not yet validated. Perform that compatibility check during the Day 3/4 interface work.
