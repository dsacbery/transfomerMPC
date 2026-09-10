classdef LateralMpcSystem < handle
    %LATERALMPCSYSTEM Fixed-dimension, fixed-parameter lateral MPC boundary.

    properties (SetAccess = private)
        Vehicle
        Config
        PreviousDelta = 0.0
        FailureCount = uint16(0)
    end

    methods
        function obj = LateralMpcSystem(vehicle, cfg)
            if ~isstruct(vehicle) || ~isstruct(cfg) || ~isfield(cfg, 'mpc') ...
                    || ~isfield(cfg, 'sample')
                error('tmpsim:InvalidLateralMpcConfig', ...
                    'vehicle and cfg must contain vehicle data and MPC settings.');
            end
            obj.Vehicle = vehicle;
            obj.Config = cfg;
        end

        function varargout = step(obj, err, meas, ref, params, resetRequested)
            if ~(islogical(resetRequested) && isscalar(resetRequested))
                error('tmpsim:InvalidLateralMpcReset', ...
                    'resetRequested must be a logical scalar.');
            end
            if resetRequested
                obj.reset();
            end

            deltaFf = 0.0;
            if isstruct(ref) && isfield(ref, 'kappa_ref') ...
                    && isFiniteScalar(ref.kappa_ref)
                deltaFf = atan((obj.Vehicle.lf + obj.Vehicle.lr) ...
                    * double(ref.kappa_ref));
            end

            if ~isValidInputs(err, meas, ref, params)
                out = obj.failureOutput(deltaFf, uint8(5));
            elseif ~isValidParams(params)
                out = obj.failureOutput(deltaFf, uint8(5), err, meas, ref);
            elseif violatesCurrentConstraints(meas, params)
                out = obj.failureOutput(deltaFf, uint8(3), err, meas, ref);
            else
                tic;
                [candidate, solved] = obj.solveCandidate(err, meas, ref, params, ...
                    deltaFf);
                elapsedMs = 1000.0 * toc;
                if solved && isfinite(candidate)
                    deltaCmd = obj.constrainCommand(candidate, params);
                    obj.PreviousDelta = deltaCmd;
                    obj.FailureCount = uint16(0);
                    out = struct( ...
                        'delta_cmd', deltaCmd, ...
                        'mpc_feasible', true, ...
                        'mpc_status_code', uint8(1), ...
                        'solve_time_ms', elapsedMs, ...
                        'delta_ff', deltaFf, ...
                        'hold_last_cmd', false);
                else
                    out = obj.failureOutput(deltaFf, uint8(3), err, meas, ref);
                    out.solve_time_ms = elapsedMs;
                end
            end

            if nargout <= 1
                varargout{1} = out;
            else
                varargout{1} = out.delta_cmd;
                varargout{2} = out;
                if nargout >= 3
                    varargout{3} = out.mpc_feasible;
                end
                if nargout >= 4
                    varargout{4} = out.mpc_status_code;
                end
                if nargout >= 5
                    varargout{5} = out.solve_time_ms;
                end
            end
        end

        function reset(obj)
            obj.PreviousDelta = 0.0;
            obj.FailureCount = uint16(0);
        end
    end

    methods (Access = private)
        function [candidate, solved] = solveCandidate(obj, err, meas, ref, params, deltaFf)
            vx = max(double(meas.vx), 1.0);
            [ad, bd, ed, cay, day, day0] = ...
                tmpsim.math.discretizeBicycleModel(obj.Vehicle, obj.Config, ...
                vx, obj.Config.sample.Ts_mpc);
            kappa = double(ref.kappa_ref);
            np = obj.Config.mpc.Np;
            nc = obj.Config.mpc.Nc;

            % The fifth state is steering correction relative to delta_ff.
            a = [ad, bd; zeros(1, 4), 1.0];
            b = [bd; 1.0];
            z0 = [double(err.e_y); double(err.e_psi); double(meas.beta); ...
                double(meas.yaw_rate); obj.PreviousDelta - deltaFf];
            knownInput = [ed * kappa + bd * deltaFf; 0.0];
            [sx, su, sk] = predictionMatrices(a, b, knownInput, np, nc);
            zBase = sx * z0 + sk;

            qz = diag([double(params.q_y), double(params.q_psi), ...
                double(params.q_beta), double(params.q_r), ...
                double(params.r_delta)]);
            qbar = kron(eye(np), qz);
            rbar = double(params.r_d_delta) * eye(nc);
            steeringOffset = repmat([zeros(4, 1); deltaFf], np, 1);
            hessian = 2.0 * (su' * qbar * su + rbar);
            hessian = 0.5 * (hessian + hessian') + 1e-9 * eye(nc);
            gradient = 2.0 * su' * qbar * (zBase + steeringOffset);
            [aIneq, bIneq] = predictionConstraints(su, zBase, deltaFf, ...
                cay, day, day0, params, obj.Config.sample.Ts_mpc, np, nc);

            options = mpcActiveSetOptions;
            [moves, exitflag] = mpcActiveSetSolver(hessian, gradient, ...
                aIneq, bIneq, zeros(0, nc), zeros(0, 1), ...
                false(size(aIneq, 1), 1), options);
            solved = exitflag > 0 && all(isfinite(moves));
            candidate = obj.PreviousDelta + moves(1);
        end

        function out = failureOutput(obj, deltaFf, statusCode, err, meas, ref)
            obj.FailureCount = obj.FailureCount + uint16(1);
            if obj.FailureCount >= uint16(3) && nargin == 6
                baseline = obj.baselineParameters();
                [candidate, solved] = obj.solveCandidate(err, meas, ref, ...
                    baseline, deltaFf);
                if solved && isfinite(candidate)
                    deltaCmd = obj.constrainCommand(candidate, baseline);
                    obj.PreviousDelta = deltaCmd;
                    obj.FailureCount = uint16(0);
                    out = struct( ...
                        'delta_cmd', deltaCmd, ...
                        'mpc_feasible', true, ...
                        'mpc_status_code', uint8(6), ...
                        'solve_time_ms', 0.0, ...
                        'delta_ff', deltaFf, ...
                        'hold_last_cmd', false);
                    return
                end
            end
            out = struct( ...
                'delta_cmd', obj.PreviousDelta, ...
                'mpc_feasible', false, ...
                'mpc_status_code', statusCode, ...
                'solve_time_ms', 0.0, ...
                'delta_ff', deltaFf, ...
                'hold_last_cmd', true);
        end

        function params = baselineParameters(obj)
            mpc = obj.Config.mpc;
            params = struct( ...
                'q_y', mpc.q_y0, ...
                'q_psi', mpc.q_psi0, ...
                'q_beta', mpc.q_beta0, ...
                'q_r', mpc.q_r0, ...
                'r_delta', mpc.r_delta0, ...
                'r_d_delta', mpc.r_d_delta0, ...
                'beta_max', mpc.beta_max0, ...
                'yaw_rate_max', mpc.yaw_rate_max0, ...
                'ay_max', mpc.ay_max0, ...
                'delta_max', mpc.delta_max, ...
                'delta_rate_max', mpc.delta_rate_max, ...
                'param_valid', true);
        end

        function deltaCmd = constrainCommand(obj, candidate, params)
            rateStep = double(params.delta_rate_max) * obj.Config.sample.Ts_mpc;
            deltaCmd = clip(candidate, obj.PreviousDelta - rateStep, ...
                obj.PreviousDelta + rateStep);
            deltaCmd = clip(deltaCmd, -double(params.delta_max), ...
                double(params.delta_max));
        end
    end
end

function valid = isValidInputs(err, meas, ref, params)
valid = isstruct(err) && isstruct(meas) && isstruct(ref) && isstruct(params);
if ~valid
    return
end
valid = all(isfield(err, {'e_y', 'e_psi', 'e_y_rate', 'e_psi_rate', ...
    'err_valid'})) ...
    && all(isfield(meas, {'vx', 'beta', 'yaw_rate', 'ay', 'meas_valid'})) ...
    && isfield(ref, 'kappa_ref') && isfield(ref, 'ref_valid');
if ~valid
    return
end
values = [err.e_y, err.e_psi, err.e_y_rate, err.e_psi_rate, ...
    meas.vx, meas.beta, meas.yaw_rate, meas.ay, ref.kappa_ref];
valid = all(cellfun(@isFiniteScalar, num2cell(values))) ...
    && islogical(err.err_valid) && isscalar(err.err_valid) && err.err_valid ...
    && islogical(meas.meas_valid) && isscalar(meas.meas_valid) ...
    && meas.meas_valid && islogical(ref.ref_valid) && isscalar(ref.ref_valid) ...
    && ref.ref_valid;
end

function valid = isValidParams(params)
names = {'q_y', 'q_psi', 'q_beta', 'q_r', 'r_delta', 'r_d_delta', ...
    'beta_max', 'yaw_rate_max', 'ay_max', 'delta_max', 'delta_rate_max'};
valid = all(isfield(params, names));
if ~valid
    return
end
values = zeros(1, numel(names));
for index = 1:numel(names)
    values(index) = double(params.(names{index}));
end
valid = all(isfinite(values)) && all(values(1:6) > 0.0) ...
    && all(values(7:end) > 0.0);
if isfield(params, 'param_valid')
    valid = valid && islogical(params.param_valid) ...
        && isscalar(params.param_valid) && params.param_valid;
end
end

function invalid = violatesCurrentConstraints(meas, params)
invalid = abs(double(meas.beta)) > double(params.beta_max) ...
    || abs(double(meas.yaw_rate)) > double(params.yaw_rate_max) ...
    || abs(double(meas.ay)) > double(params.ay_max);
end

function valid = isFiniteScalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) ...
    && isfinite(value);
end

function value = clip(value, lowerBound, upperBound)
value = min(max(value, lowerBound), upperBound);
end

function [sx, su, sk] = predictionMatrices(a, b, knownInput, np, nc)
nz = size(a, 1);
sx = zeros(np * nz, nz);
sk = zeros(np * nz, 1);
fullMoveResponse = zeros(np * nz, np);
for step = 1:np
    row = (step - 1) * nz + (1:nz);
    sx(row, :) = a^step;
    known = zeros(nz, 1);
    for inputStep = 1:step
        known = known + a^(step - inputStep) * knownInput;
        fullMoveResponse(row, inputStep) = a^(step - inputStep) * b;
    end
    sk(row) = known;
end
moveBlocking = [eye(nc); zeros(np - nc, nc)];
su = fullMoveResponse * moveBlocking;
end

function [aIneq, bIneq] = predictionConstraints(su, zBase, deltaFf, ...
        cay, day, day0, params, tsMpc, np, nc)
nz = 5;
selectors = [ ...
    0 0 0 0 1; ...
    0 0 1 0 0; ...
    0 0 0 1 0; ...
    cay, day];
limits = [double(params.delta_max); double(params.beta_max); ...
    double(params.yaw_rate_max); double(params.ay_max)];
offsets = [deltaFf; 0.0; 0.0; day0];
aIneq = [eye(nc); -eye(nc)];
rateLimit = double(params.delta_rate_max) * tsMpc;
bIneq = rateLimit * ones(2 * nc, 1);
for step = 1:np
    rows = (step - 1) * nz + (1:nz);
    for index = 1:numel(limits)
        projection = selectors(index, :) * su(rows, :);
        base = selectors(index, :) * zBase(rows) + offsets(index);
        aIneq = [aIneq; projection; -projection]; %#ok<AGROW>
        bIneq = [bIneq; limits(index) - base; limits(index) + base]; %#ok<AGROW>
    end
end
end
