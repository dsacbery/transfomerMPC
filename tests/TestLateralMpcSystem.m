classdef TestLateralMpcSystem < matlab.unittest.TestCase
    %TESTLATERALMPCSYSTEM Verifies the Day 7 fixed-parameter lateral MPC.

    properties (Constant)
        TsMpc = 0.02
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture(fullfile(projectRoot, ...
                'src')));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture(fullfile(projectRoot, ...
                'config')));
        end
    end

    methods (Test)
        function testDiscretizedBicycleModelHasFrozenDimensions(testCase)
            cfg = tmpsim_config();
            vehicle = vehicle_params();

            [Ad, Bd, Ed, Cay, Day, day0] = ...
                tmpsim.math.discretizeBicycleModel(vehicle, cfg, 8.0, ...
                testCase.TsMpc);

            testCase.verifySize(Ad, [4 4]);
            testCase.verifySize(Bd, [4 1]);
            testCase.verifySize(Ed, [4 1]);
            testCase.verifySize(Cay, [1 4]);
            testCase.verifySize(Day, [1 1]);
            testCase.verifySize(day0, [1 1]);
            testCase.verifyClass(Ad, 'double');
            testCase.verifyTrue(all(isfinite([Ad(:); Bd(:); Ed(:); ...
                Cay(:); Day(:); day0(:)])));
            testCase.verifyGreaterThan(norm(Ad - eye(4), 'fro'), 0.0);
        end

        function testStraightNominalReturnsFiniteSolvedOutput(testCase)
            [controller, cfg, ~] = newController();
            out = controller.step(zeroError(), validMeasurement(8.0), ...
                reference(0.0), baselineParams(cfg), false);

            testCase.verifyEqual(fieldnames(out).', { ...
                'delta_cmd', 'mpc_feasible', 'mpc_status_code', ...
                'solve_time_ms', 'delta_ff', 'hold_last_cmd'});
            testCase.verifyEqual(out.delta_cmd, 0.0, AbsTol=1e-10);
            testCase.verifyTrue(out.mpc_feasible);
            testCase.verifyEqual(out.mpc_status_code, uint8(1));
            testCase.verifyTrue(isfinite(out.solve_time_ms));
            testCase.verifyGreaterThanOrEqual(out.solve_time_ms, 0.0);
            testCase.verifyFalse(out.hold_last_cmd);
        end

        function testCircularReferenceProducesCurvatureFeedforward(testCase)
            [controller, cfg, vehicle] = newController();
            curvature = 0.02;
            out = controller.step(zeroError(), validMeasurement(10.0), ...
                reference(curvature), baselineParams(cfg), false);

            expected = atan((vehicle.lf + vehicle.lr) * curvature);
            testCase.verifyEqual(out.delta_ff, expected, AbsTol=1e-12);
            testCase.verifyTrue(out.mpc_feasible);
            testCase.verifyLessThanOrEqual(abs(out.delta_cmd), ...
                cfg.mpc.delta_max + 1e-12);
        end

        function testDynamicWeightsRemainFiniteAndAffectSolve(testCase)
            [controller, cfg, ~] = newController();
            err = zeroError();
            err.e_y = 0.01;
            base = baselineParams(cfg);
            baseOut = controller.step(err, validMeasurement(8.0), ...
                reference(0.0), base, false);

            dynamic = base;
            dynamic.q_y = cfg.mpc.q_y_max;
            dynamic.q_psi = cfg.mpc.q_psi_max;
            dynamic.q_beta = cfg.mpc.q_beta_max;
            dynamic.q_r = cfg.mpc.q_r_max;
            dynamic.r_delta = cfg.mpc.r_delta_max;
            dynamic.r_d_delta = cfg.mpc.r_d_delta_max;
            dynamicOut = controller.step(err, validMeasurement(8.0), ...
                reference(0.0), dynamic, true);

            testCase.verifyTrue(dynamicOut.mpc_feasible);
            testCase.verifyTrue(all(isfinite([baseOut.delta_cmd, ...
                dynamicOut.delta_cmd, dynamicOut.solve_time_ms])));
            testCase.verifyGreaterThan(abs(dynamicOut.delta_cmd - ...
                baseOut.delta_cmd), 1e-12);
        end

        function testSteeringAmplitudeAndRateConstraintsAreEnforced(testCase)
            [controller, cfg, ~] = newController();
            params = baselineParams(cfg);
            params.delta_max = 0.05;
            params.delta_rate_max = 0.10;
            err = zeroError();
            err.e_y = 5.0;

            first = controller.step(err, validMeasurement(8.0), ...
                reference(0.0), params, false);
            second = controller.step(err, validMeasurement(8.0), ...
                reference(0.0), params, false);

            testCase.verifyLessThanOrEqual(abs(first.delta_cmd), ...
                params.delta_max + 1e-12);
            testCase.verifyLessThanOrEqual(abs(second.delta_cmd), ...
                params.delta_max + 1e-12);
            testCase.verifyLessThanOrEqual(abs(second.delta_cmd - ...
                first.delta_cmd), params.delta_rate_max * testCase.TsMpc + 1e-12);
        end

        function testInvalidInputHoldsPreviousCommand(testCase)
            [controller, cfg, ~] = newController();
            params = baselineParams(cfg);
            first = controller.step(zeroError(), validMeasurement(8.0), ...
                reference(0.0), params, false);
            invalid = validMeasurement(8.0);
            invalid.meas_valid = false;
            out = controller.step(zeroError(), invalid, reference(0.0), ...
                params, false);

            testCase.verifyFalse(out.mpc_feasible);
            testCase.verifyEqual(out.mpc_status_code, uint8(5));
            testCase.verifyTrue(out.hold_last_cmd);
            testCase.verifyEqual(out.delta_cmd, first.delta_cmd, AbsTol=1e-12);
            testCase.verifyTrue(isfinite(out.delta_cmd));
        end

        function testInvalidErrorHoldsPreviousCommand(testCase)
            [controller, cfg, ~] = newController();
            params = baselineParams(cfg);
            first = controller.step(zeroError(), validMeasurement(8.0), ...
                reference(0.0), params, false);
            invalid = zeroError();
            invalid.err_valid = false;
            out = controller.step(invalid, validMeasurement(8.0), ...
                reference(0.0), params, false);

            testCase.verifyFalse(out.mpc_feasible);
            testCase.verifyEqual(out.mpc_status_code, uint8(5));
            testCase.verifyTrue(out.hold_last_cmd);
            testCase.verifyEqual(out.delta_cmd, first.delta_cmd, AbsTol=1e-12);
        end

        function testThirdFailureRetriesWithBaselineParameters(testCase)
            [controller, cfg, ~] = newController();
            invalidParams = baselineParams(cfg);
            invalidParams.param_valid = false;

            first = controller.step(zeroError(), validMeasurement(8.0), ...
                reference(0.0), invalidParams, false);
            second = controller.step(zeroError(), validMeasurement(8.0), ...
                reference(0.0), invalidParams, false);
            third = controller.step(zeroError(), validMeasurement(8.0), ...
                reference(0.0), invalidParams, false);

            testCase.verifyEqual(first.mpc_status_code, uint8(5));
            testCase.verifyEqual(second.mpc_status_code, uint8(5));
            testCase.verifyTrue(third.mpc_feasible);
            testCase.verifyEqual(third.mpc_status_code, uint8(6));
            testCase.verifyFalse(third.hold_last_cmd);
            testCase.verifyEqual(third.delta_cmd, 0.0, AbsTol=1e-12);
        end

        function testInfeasibleStateConstraintsUseSafeFallback(testCase)
            [controller, cfg, ~] = newController();
            params = baselineParams(cfg);
            params.beta_max = 1e-3;
            meas = validMeasurement(8.0);
            meas.beta = 0.5;
            out = controller.step(zeroError(), meas, reference(0.0), ...
                params, false);

            testCase.verifyFalse(out.mpc_feasible);
            testCase.verifyTrue(ismember(out.mpc_status_code, ...
                uint8([3 6])));
            testCase.verifyTrue(out.hold_last_cmd);
            testCase.verifyTrue(isfinite(out.delta_cmd));
        end

        function testResetClearsHeldCommand(testCase)
            [controller, cfg, ~] = newController();
            params = baselineParams(cfg);
            err = zeroError();
            err.e_y = 0.5;
            controller.step(err, validMeasurement(8.0), reference(0.0), ...
                params, false);
            out = controller.step(zeroError(), validMeasurement(8.0), ...
                reference(0.0), params, true);

            testCase.verifyEqual(out.delta_cmd, 0.0, AbsTol=1e-12);
            testCase.verifyTrue(out.mpc_feasible);
            testCase.verifyEqual(out.mpc_status_code, uint8(1));
        end
    end
end

function [controller, cfg, vehicle] = newController()
cfg = tmpsim_config();
vehicle = vehicle_params();
controller = tmpsim.LateralMpcSystem(vehicle, cfg);
end

function params = baselineParams(cfg)
params = struct( ...
    'q_y', cfg.mpc.q_y0, ...
    'q_psi', cfg.mpc.q_psi0, ...
    'q_beta', cfg.mpc.q_beta0, ...
    'q_r', cfg.mpc.q_r0, ...
    'r_delta', cfg.mpc.r_delta0, ...
    'r_d_delta', cfg.mpc.r_d_delta0, ...
    'beta_max', cfg.mpc.beta_max0, ...
    'yaw_rate_max', cfg.mpc.yaw_rate_max0, ...
    'ay_max', cfg.mpc.ay_max0, ...
    'delta_max', cfg.mpc.delta_max, ...
    'delta_rate_max', cfg.mpc.delta_rate_max, ...
    'param_valid', true);
end

function err = zeroError()
err = struct('e_y', 0.0, 'e_psi', 0.0, 'e_y_rate', 0.0, ...
    'e_psi_rate', 0.0, 'err_valid', true);
end

function meas = validMeasurement(vx)
meas = struct('vx', vx, 'beta', 0.0, 'yaw_rate', 0.0, 'ay', 0.0, ...
    'meas_valid', true);
end

function ref = reference(kappa)
ref = struct('kappa_ref', kappa, 'ref_valid', true);
end
