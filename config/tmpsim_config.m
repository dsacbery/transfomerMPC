function cfg = tmpsim_config()
%TMPSIM_CONFIG Returns the frozen P0 online-controller configuration.

cfg.environment.matlab_release = "R" + string(version('-release'));
cfg.environment.carsim_version = "2019.1";
cfg.environment.carsim_simulink_verified = false;

cfg.convention.delta_positive = "left";
cfg.convention.yaw_positive = "left";
cfg.convention.ay_positive = "left";

cfg.sample.Ts_mpc = 0.02;
cfg.sample.Ts_tr = 0.10;

cfg.history.L = 16;
cfg.history.n_feature = 14;
cfg.feature_order = [ ...
    "vx", "vy", "yaw_rate", "ay", "beta", "delta_meas", ...
    "delta_rate_meas", "ax_meas", "e_y", "e_psi", ...
    "e_y_rate", "e_psi_rate", "kappa_ref", "v_ref_base"];

cfg.mpc.Np = 15;
cfg.mpc.Nc = 5;
cfg.mpc.delta_max = deg2rad(35);
cfg.mpc.delta_rate_max = deg2rad(450);
cfg.mpc.beta_max0 = deg2rad(8);
cfg.mpc.yaw_rate_max0 = 1.2;
cfg.mpc.ay_max0 = 6.0;
cfg.mpc.q_y0 = 12.0;
cfg.mpc.q_psi0 = 8.0;
cfg.mpc.q_beta0 = 4.0;
cfg.mpc.q_r0 = 2.0;
cfg.mpc.r_delta0 = 0.8;
cfg.mpc.r_d_delta0 = 0.2;
cfg.mpc.q_y_max = 60.0;
cfg.mpc.q_psi_max = 40.0;
cfg.mpc.q_beta_max = 30.0;
cfg.mpc.q_r_max = 20.0;
cfg.mpc.r_delta_max = 8.0;
cfg.mpc.r_d_delta_max = 5.0;
cfg.mpc.c_beta = 0.45;
cfg.mpc.c_yaw_rate = 0.35;
cfg.mpc.c_ay = 0.30;
cfg.mpc.beta_min = deg2rad(2);
cfg.mpc.yaw_rate_min = 0.4;
cfg.mpc.ay_min = 2.5;
cfg.mpc.delta_rate_min = deg2rad(120);

cfg.risk.k_v_min = 0.35;
cfg.risk.rho_up = 0.25;
cfg.risk.rho_down = 0.90;
cfg.risk.fail_cycles = uint16(3);
cfg.risk.recover_cycles = uint16(10);
cfg.risk.e_y_warn = 0.30;
cfg.risk.beta_warn = deg2rad(5);
cfg.risk.yaw_rate_warn = 0.8;
cfg.risk.ay_warn = 4.0;
cfg.risk.c_v_rule = 0.50;
cfg.risk.c_v = 0.50;

cfg.speed.v_min = 1.0;
cfg.speed.a_min = -6.0;
cfg.speed.a_max = 2.5;
cfg.speed.jerk_max = 8.0;

cfg.transformer.input_size = [cfg.history.L cfg.history.n_feature];
cfg.transformer.output_names = ["r_low", "r_ey", "r_stab", "k_v"];
cfg.transformer.output_bounds = [0 1; 0 1; 0 1; cfg.risk.k_v_min 1];

cfg.control_mode.pid = uint8(0);
cfg.control_mode.fixed_mpc = uint8(1);
cfg.control_mode.rule_risk_mpc = uint8(2);
cfg.control_mode.transformer_mpc = uint8(3);

cfg.status.transformer.reset = uint8(0);
cfg.status.transformer.idle = uint8(1);
cfg.status.transformer.ok = uint8(2);
cfg.status.transformer.fallback = uint8(3);
cfg.status.supervisor.normal = uint8(0);
cfg.status.supervisor.fallback = uint8(1);
cfg.status.supervisor.recovering = uint8(2);
cfg.status.mpc.not_run = uint8(0);
cfg.status.mpc.solved = uint8(1);
cfg.status.mpc.warm_start_solved = uint8(2);
cfg.status.mpc.infeasible = uint8(3);
cfg.status.mpc.timeout = uint8(4);
cfg.status.mpc.invalid_input = uint8(5);
cfg.status.mpc.fixed_mpc_degraded = uint8(6);
cfg.status.speed.reset = uint8(0);
cfg.status.speed.normal = uint8(1);
cfg.status.speed.invalid_measurement = uint8(2);

cfg.simulink.Ts_mpc = Simulink.Parameter(cfg.sample.Ts_mpc);
cfg.simulink.Ts_mpc.DataType = 'double';
cfg.simulink.Ts_tr = Simulink.Parameter(cfg.sample.Ts_tr);
cfg.simulink.Ts_tr.DataType = 'double';
end
