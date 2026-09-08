# CarSim 2019.1 Signal Inventory

Verified against the `day2_export_smoke` run of the CarSim Quick Start DLC example.

| MeasBus field     | CarSim source or derivation         | CarSim unit | Internal unit | Conversion or validity rule              |
| ----------------- | ----------------------------------- | -----------:| -------------:| ---------------------------------------- |
| `x`               | `Xo`                                | m           | m             | pass through                             |
| `y`               | `Yo`                                | m           | m             | pass through                             |
| `yaw`             | `Yaw`                               | deg         | rad           | `deg2rad(Yaw)`                           |
| `vx`              | `Vx`                                | km/h        | m/s           | `Vx / 3.6`                               |
| `vy`              | `Vy`                                | km/h        | m/s           | `Vy / 3.6`                               |
| `yaw_rate`        | `AVz`                               | deg/s       | rad/s         | `deg2rad(AVz)`                           |
| `beta`            | `Beta`                              | deg         | rad           | `deg2rad(Beta)`                          |
| `ay`              | `Ay`                                | g           | m/s^2         | `Ay * 9.80665`                           |
| `delta_meas`      | `Steer_L1`, `Steer_R1`              | deg         | rad           | `deg2rad((Steer_L1 + Steer_R1) / 2)`     |
| `ax_meas`         | `Ax`                                | g           | m/s^2         | `Ax * 9.80665`                           |
| `delta_rate_meas` | discrete derivative of `delta_meas` | rad/s       | rad/s         | `(delta_meas(k)-delta_meas(k-1))/Ts_mpc` |
| `meas_valid`      | local validity check                | boolean     | boolean       | all mapped raw values are finite         |

`StrAV_SW` is steering-wheel angular rate and is retained only as a diagnostic signal. It must not be used as `delta_rate_meas` unless a validated steering-ratio and sign conversion is introduced.

The CarSim-to-internal sign convention for a known left turn remains to be verified in the Day 4 interface test. The internal convention is `delta > 0` for a front-wheel left turn.
