function validateOnlineConfig(cfg, featureStats)
%VALIDATEONLINECONFIG Rejects inconsistent P0 online-controller settings.

if nargin < 2
    featureStats = [];
end

validateRequiredFields(cfg, {'sample', 'history', 'feature_order', 'mpc', ...
    'risk', 'speed'});
validateRequiredFields(cfg.sample, {'Ts_mpc', 'Ts_tr'});
validatePositiveFinite(cfg.sample.Ts_mpc, 'Ts_mpc');
validatePositiveFinite(cfg.sample.Ts_tr, 'Ts_tr');

rateRatio = cfg.sample.Ts_tr / cfg.sample.Ts_mpc;
if abs(rateRatio - round(rateRatio)) > 1e-12
    error('tmpsim:InvalidSampleRateRatio', ...
        'Ts_tr/Ts_mpc must be an integer ratio.');
end

validateRequiredFields(cfg.history, {'L', 'n_feature'});
if cfg.history.L ~= 16 || cfg.history.n_feature ~= 14
    error('tmpsim:InvalidHistoryShape', ...
        'P0 requires a 16-by-14 Transformer history window.');
end

expectedFeatureOrder = [ ...
    "vx", "vy", "yaw_rate", "ay", "beta", "delta_meas", ...
    "delta_rate_meas", "ax_meas", "e_y", "e_psi", ...
    "e_y_rate", "e_psi_rate", "kappa_ref", "v_ref_base"];
if ~isequal(string(cfg.feature_order), expectedFeatureOrder)
    error('tmpsim:InvalidFeatureOrder', ...
        'feature_order must match the frozen 14-column signal contract.');
end

validateMpcBounds(cfg.mpc);
validateRiskBounds(cfg.risk);
validateSpeedBounds(cfg.speed);

if ~isempty(featureStats)
    validateFeatureStats(featureStats, expectedFeatureOrder);
end
end

function validateMpcBounds(mpc)
validateRequiredFields(mpc, {'Np', 'Nc', 'delta_max', 'delta_rate_max', ...
    'beta_max0', 'yaw_rate_max0', 'ay_max0', 'q_y0', 'q_psi0', ...
    'q_beta0', 'q_r0', 'r_delta0', 'r_d_delta0', 'q_y_max', ...
    'q_psi_max', 'q_beta_max', 'q_r_max', 'r_delta_max', ...
    'r_d_delta_max', 'beta_min', 'yaw_rate_min', 'ay_min', ...
    'delta_rate_min'});

if mpc.Np ~= 15 || mpc.Nc ~= 5 || mpc.Nc > mpc.Np
    error('tmpsim:InvalidMpcHorizon', ...
        'P0 requires Np=15, Nc=5, and Nc no greater than Np.');
end

validatePositiveFinite([mpc.delta_max, mpc.delta_rate_max, mpc.beta_max0, ...
    mpc.yaw_rate_max0, mpc.ay_max0], 'MPC limits');
validatePositiveFinite([mpc.q_y0, mpc.q_psi0, mpc.q_beta0, mpc.q_r0, ...
    mpc.r_delta0, mpc.r_d_delta0, mpc.q_y_max, mpc.q_psi_max, ...
    mpc.q_beta_max, mpc.q_r_max, mpc.r_delta_max, mpc.r_d_delta_max], ...
    'MPC weights');

validateRange(mpc.beta_min, mpc.beta_max0, 'beta bounds');
validateRange(mpc.yaw_rate_min, mpc.yaw_rate_max0, 'yaw-rate bounds');
validateRange(mpc.ay_min, mpc.ay_max0, 'lateral-acceleration bounds');
validateRange(mpc.delta_rate_min, mpc.delta_rate_max, ...
    'steering-rate bounds');
validateRange(mpc.q_y0, mpc.q_y_max, 'q_y bounds');
validateRange(mpc.q_psi0, mpc.q_psi_max, 'q_psi bounds');
validateRange(mpc.q_beta0, mpc.q_beta_max, 'q_beta bounds');
validateRange(mpc.q_r0, mpc.q_r_max, 'q_r bounds');
validateRange(mpc.r_delta0, mpc.r_delta_max, 'r_delta bounds');
validateRange(mpc.r_d_delta0, mpc.r_d_delta_max, 'r_d_delta bounds');
end

function validateRiskBounds(risk)
validateRequiredFields(risk, {'k_v_min', 'rho_up', 'rho_down', ...
    'fail_cycles', 'recover_cycles', 'e_y_warn', 'beta_warn', ...
    'yaw_rate_warn', 'ay_warn', 'c_v_rule', 'c_v'});
validateRange(risk.k_v_min, 1, 'k_v bounds');
validateRange(risk.rho_up, 1, 'rho_up bounds');
validateRange(risk.rho_down, 1, 'rho_down bounds');
if risk.rho_up >= risk.rho_down
    error('tmpsim:InvalidRiskFilter', ...
        'rho_up must be lower than rho_down for fast rise and slow decay.');
end
validatePositiveFinite([risk.e_y_warn, risk.beta_warn, ...
    risk.yaw_rate_warn, risk.ay_warn], 'risk thresholds');
validateRange(risk.c_v_rule, 1, 'c_v_rule bounds');
validateRange(risk.c_v, 1, 'c_v bounds');
if risk.fail_cycles < 1 || risk.recover_cycles < 1
    error('tmpsim:InvalidRiskCounters', ...
        'Risk failure and recovery counts must be positive.');
end
end

function validateSpeedBounds(speed)
validateRequiredFields(speed, {'v_min', 'a_min', 'a_max', 'jerk_max'});
validatePositiveFinite([speed.v_min, speed.jerk_max], 'speed limits');
validateRange(speed.a_min, speed.a_max, 'longitudinal-acceleration bounds');
end

function validateFeatureStats(featureStats, expectedFeatureOrder)
validateRequiredFields(featureStats, {'mean_train', 'std_train', ...
    'feature_order'});
if ~isequal(size(featureStats.mean_train), [1 14]) || ...
        ~isequal(size(featureStats.std_train), [1 14])
    error('tmpsim:InvalidFeatureStatsSize', ...
        'mean_train and std_train must both be 1-by-14.');
end
if any(~isfinite(featureStats.mean_train)) || ...
        any(~isfinite(featureStats.std_train)) || ...
        any(featureStats.std_train <= 0)
    error('tmpsim:InvalidFeatureStatsValues', ...
        'Feature statistics must be finite and std_train must be positive.');
end
if ~isequal(string(featureStats.feature_order), expectedFeatureOrder)
    error('tmpsim:InvalidFeatureOrder', ...
        'Feature-statistics order must match the frozen signal contract.');
end
end

function validateRequiredFields(value, fieldNames)
if ~isstruct(value) || ~all(isfield(value, fieldNames))
    error('tmpsim:InvalidConfigStructure', ...
        'Configuration is missing one or more required fields.');
end
end

function validatePositiveFinite(value, name)
if ~isnumeric(value) || any(~isfinite(value), 'all') || any(value <= 0, 'all')
    error('tmpsim:InvalidPositiveValue', ...
        '%s must contain finite positive values.', name);
end
end

function validateRange(lowerBound, upperBound, name)
if ~isnumeric(lowerBound) || ~isnumeric(upperBound) || ...
        ~isscalar(lowerBound) || ~isscalar(upperBound) || ...
        ~isfinite(lowerBound) || ~isfinite(upperBound) || ...
        lowerBound > upperBound
    error('tmpsim:InvalidBounds', ...
        'Invalid %s.', name);
end
end
