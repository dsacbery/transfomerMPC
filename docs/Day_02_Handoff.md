# Day 2 Handoff: Signal Contract and Configuration

## Current Status

Day 2 is partially complete. The Simulink Data Dictionary and all eight Bus contracts are implemented and verified. The two required configuration files and executable feature-order metadata are not yet implemented, so Day 2 must not be marked complete.

## Completed and Verified

- MATLAB R2024b has Simulink, Deep Learning Toolbox, and Model Predictive Control Toolbox. `mpcActiveSetSolver` and `dlnetwork` are available.
- CarSim 2019.1 completed the `day2_export_smoke` copy of the Quick Start `DLC @ 120 km/h` case and produced usable plots.
- The internal `MeasBus` mapping and SI conversions are recorded in `docs/CarSim_2019_1_Signal_Inventory.md`.
- `model/tmpsim_online.sldd` contains `RefBus`, `MeasBus`, `ErrBus`, `HistoryBus`, `RiskRawBus`, `RiskBus`, `MPCParamBus`, and `CmdBus`.
- `HistoryBus.window` is `16x14 double`; continuous values are `double`, flags are `boolean`, and status or mode IDs are `uint8`.
- `scripts/create_tmpsim_dictionary.m` recreates the Data Dictionary contract deterministically.
- `tests/TestSignalContracts.m` contains five passing MATLAB contract tests.

Run the verification from the project root:

```matlab
cd('D:\Desktop\TransformerMPC')
addpath('scripts')
create_tmpsim_dictionary()

results = runtests('tests/TestSignalContracts.m');
assertSuccess(results)
```

## Remaining Day 2 Work

1. Create `config/tmpsim_config.m`.
   - Return the `cfg` structure.
   - Freeze `Ts_mpc=0.02`, `Ts_tr=0.10`, `history.L=16`, `history.n_feature=14`, `Np=15`, and `Nc=5`.
   - Store the 14-element feature order as executable metadata, not only in Markdown.
   - Include the P0 MPC, risk, and speed baseline bounds from the implementation blueprint.
   - Record `MATLAB R2024b` and `CarSim 2019.1` as the actual local environment; do not put absolute paths or license information in the configuration.

2. Create `config/vehicle_params.m` from the C-Class Hatchback used by `day2_export_smoke`.
   - The expanded CarSim result gives total mass `1501 kg`, yaw inertia `2192.089539 kg*m^2`, wheelbase `2.910 m`, `lf=1.049562958 m`, and `lr=1.860437042 m`.
   - The nominal small-slip tire-table linearization gives candidate axle cornering stiffnesses `Cf=159055.11037704 N/rad` and `Cr=92776.4754843415 N/rad`.
   - Treat `Cf` and `Cr` as nominal linearization values. Before the Day 7 lateral MPC is accepted, verify the operating-point selection and sign convention against CarSim. Do not replace them with guessed values.

3. Extend `TestSignalContracts.m` after configuration files exist.
   - Verify the executable 14-column feature order exactly matches the frozen interface:

```text
vx, vy, yaw_rate, ay, beta, delta_meas, delta_rate_meas,
ax_meas, e_y, e_psi, e_y_rate, e_psi_rate, kappa_ref, v_ref_base
```

   - Verify `Ts_tr/Ts_mpc` is an integer and the required `cfg` bounds are internally consistent.

4. Complete the remaining manual CarSim convention check.
   - Verify, using a known left-turn maneuver, that the sign of `Steer_L1`/`Steer_R1`, `Yaw`, `AVz`, and `Ay` is consistent with the internal rule `delta > 0` for a front-wheel left turn.
   - Keep the CarSim raw-to-SI conversions in `docs/CarSim_2019_1_Signal_Inventory.md` as the single source of truth.

## Do Not Start Yet

- Do not create `model/tmpsim_online.slx`; that is Day 3.
- Do not implement `CarSimAdapter.m`; that is Day 4.
- Do not connect the Transformer, Risk Supervisor, or MPC before the fixed-MPC baseline stage.
- Do not add CarSim installation files, generated results, caches, or license files to this repository.

## Local CarSim Context

- CarSim program: `D:\Carsim\carsim2019.1`.
- Active local database: `D:\Desktop\Carsim`.
- Evidence run: `day2_export_smoke`, based on Quick Start `DLC @ 120 km/h` and C-Class Hatchback.
- The CarSim 2019.1 and MATLAB R2024b Simulink interface has not yet been validated. Perform that compatibility check during the Day 3/4 interface work.
