classdef TestErrorFeatureBuilder < matlab.unittest.TestCase
    %TESTERRORFEATUREBUILDER Verifies frozen Frenet errors and feature order.

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture(fullfile(projectRoot, ...
                'src')));
        end
    end

    methods (Test)
        function testRejectsNonFrozenMpcSampleTime(testCase)
            testCase.verifyError( ...
                @() tmpsim.ErrorFeatureBuilder(0.01), ...
                'tmpsim:InvalidErrorFeatureSampleTime');
        end

        function testBuildsZeroStraightPathErrorsInFrozenFeatureOrder(testCase)
            builder = tmpsim.ErrorFeatureBuilder(0.02);
            ref = reference(10.0, 0.0, 0.0, 0.0, 8.0, true);
            meas = measurement(10.0, 0.0, 0.0, 8.0, 0.0, 0.0, ...
                0.0, 0.0, 0.1, 0.2, 0.3, true);

            [err, latestFeature] = builder.build(ref, meas, false);

            testCase.verifyEqual(err.e_y, 0.0, AbsTol=1e-12);
            testCase.verifyEqual(err.e_psi, 0.0, AbsTol=1e-12);
            testCase.verifyEqual(err.e_y_rate, 0.0, AbsTol=1e-12);
            testCase.verifyEqual(err.e_psi_rate, 0.0, AbsTol=1e-12);
            testCase.verifyTrue(err.err_valid);
            testCase.verifyEqual(latestFeature, [ ...
                meas.vx, meas.vy, meas.yaw_rate, meas.ay, meas.beta, ...
                meas.delta_meas, meas.delta_rate_meas, meas.ax_meas, ...
                0.0, 0.0, 0.0, 0.0, 0.0, 8.0], AbsTol=1e-12);
            testCase.verifySize(latestFeature, [1 14]);
            testCase.verifyClass(latestFeature, 'double');
            testCase.verifyTrue(all(isfinite(latestFeature)));
            testCase.verifyEqual(fieldnames(err).', { ...
                'e_y', 'e_psi', 'e_y_rate', 'e_psi_rate', 'yaw_ref', ...
                'kappa_ref', 'v_ref_base', 'err_valid'});
        end

        function testComputesLeftPositiveFrenetLateralError(testCase)
            builder = tmpsim.ErrorFeatureBuilder(0.02);
            ref = reference(0.0, 0.0, pi / 2.0, 0.0, 6.0, true);
            meas = measurement(-2.0, 0.0, pi / 2.0, 6.0, 0.0, 0.0, ...
                0.0, 0.0, 0.0, 0.0, 0.0, true);

            [err, ~] = builder.build(ref, meas, false);

            testCase.verifyEqual(err.e_y, 2.0, AbsTol=1e-12);
            testCase.verifyTrue(err.err_valid);
        end

        function testWrapsHeadingErrorAcrossPiBoundary(testCase)
            builder = tmpsim.ErrorFeatureBuilder(0.02);
            ref = reference(0.0, 0.0, -pi + 0.01, 0.0, 6.0, true);
            meas = measurement(0.0, 0.0, pi - 0.01, 6.0, 0.0, 0.0, ...
                0.0, 0.0, 0.0, 0.0, 0.0, true);

            [err, ~] = builder.build(ref, meas, false);

            testCase.verifyEqual(err.e_psi, -0.02, AbsTol=1e-12);
        end

        function testComputesCurvatureAwareErrorRates(testCase)
            builder = tmpsim.ErrorFeatureBuilder(0.02);
            ref = reference(0.0, 0.0, 0.0, 0.1, 8.0, true);
            meas = measurement(0.0, 0.0, pi / 6.0, 10.0, 1.0, 0.4, ...
                0.0, 0.0, 0.0, 0.0, 0.0, true);

            [err, ~] = builder.build(ref, meas, false);

            testCase.verifyEqual(err.e_y_rate, 6.0, AbsTol=1e-12);
            testCase.verifyEqual(err.e_psi_rate, -0.6, AbsTol=1e-12);
            testCase.verifyTrue(err.err_valid);
        end

        function testUsesClippedFiniteDifferenceBelowSpeedThreshold(testCase)
            builder = tmpsim.ErrorFeatureBuilder(0.02);
            ref = reference(0.0, 0.0, 0.0, 0.0, 6.0, true);

            [first, ~] = builder.build(ref, measurement(0.0, 1.0, 0.0, ...
                0.49, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true), false);
            [second, ~] = builder.build(ref, measurement(0.0, 1.02, 0.0, ...
                0.49, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true), false);

            testCase.verifyEqual(first.e_y_rate, 5.0, AbsTol=1e-12);
            testCase.verifyEqual(second.e_y_rate, 1.0, AbsTol=1e-12);
            testCase.verifyFalse(first.err_valid);
            testCase.verifyFalse(second.err_valid);
        end

        function testResetClearsLowSpeedFiniteDifferenceState(testCase)
            builder = tmpsim.ErrorFeatureBuilder(0.02);
            ref = reference(0.0, 0.0, 0.0, 0.0, 6.0, true);
            slowMeasurement = measurement(0.0, 1.02, 0.0, 0.49, 0.0, ...
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true);

            builder.build(ref, measurement(0.0, 1.0, 0.0, 0.49, 0.0, ...
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true), false);
            [beforeReset, ~] = builder.build(ref, slowMeasurement, false);
            [afterReset, ~] = builder.build(ref, slowMeasurement, true);

            testCase.verifyEqual(beforeReset.e_y_rate, 1.0, AbsTol=1e-12);
            testCase.verifyEqual(afterReset.e_y_rate, 5.0, AbsTol=1e-12);
            testCase.verifyFalse(afterReset.err_valid);
        end

        function testInvalidBusesProduceFiniteInvalidOutputsWithoutStateUpdate(testCase)
            builder = tmpsim.ErrorFeatureBuilder(0.02);
            ref = reference(0.0, 0.0, 0.0, 0.0, 6.0, true);
            builder.build(ref, measurement(0.0, 1.0, 0.0, 0.49, 0.0, ...
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true), false);
            invalidMeasurement = measurement(NaN, 0.0, 0.0, 0.49, 0.0, ...
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);

            [invalidErr, invalidFeature] = builder.build(ref, invalidMeasurement, false);
            [recoveredErr, recoveredFeature] = builder.build(ref, ...
                measurement(0.0, 1.02, 0.0, 0.49, 0.0, 0.0, 0.0, ...
                0.0, 0.0, 0.0, 0.0, true), false);

            testCase.verifyFalse(invalidErr.err_valid);
            testCase.verifyEqual(invalidFeature, zeros(1, 14), AbsTol=1e-12);
            testCase.verifyTrue(all(isfinite(invalidFeature)));
            testCase.verifyEqual(recoveredErr.e_y_rate, 1.0, AbsTol=1e-12);
            testCase.verifyTrue(all(isfinite(recoveredFeature)));
        end

        function testReferenceManagerFeedsLaneChangeErrorDataPath(testCase)
            path = laneChangePath();
            manager = tmpsim.ReferenceManager(path, 10.0, 1.0);
            builder = tmpsim.ErrorFeatureBuilder(0.02);
            measForReference = struct('x', 60.0, 'y', laneChangeY(60.0), ...
                'meas_valid', true);
            ref = manager.step(measForReference, false);
            meas = measurement(ref.x_ref, ref.y_ref, ref.psi_ref, 8.0, ...
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true);

            [err, latestFeature] = builder.build(ref, meas, false);

            testCase.verifyTrue(ref.ref_valid);
            testCase.verifyEqual(err.e_y, 0.0, AbsTol=1e-12);
            testCase.verifyEqual(err.e_psi, 0.0, AbsTol=1e-12);
            testCase.verifyTrue(err.err_valid);
            testCase.verifyEqual(latestFeature(13), ref.kappa_ref, AbsTol=1e-12);
            testCase.verifyEqual(latestFeature(14), ref.v_ref_base, AbsTol=1e-12);
        end

        function testInvalidReferenceProducesFiniteInvalidOutputs(testCase)
            builder = tmpsim.ErrorFeatureBuilder(0.02);
            invalidRef = reference(0.0, 0.0, 0.0, 0.0, 6.0, false);
            meas = measurement(0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, ...
                0.0, 0.0, 0.0, 0.0, true);

            [err, latestFeature] = builder.build(invalidRef, meas, false);

            testCase.verifyFalse(err.err_valid);
            testCase.verifyTrue(all(isfinite(struct2array(err))));
            testCase.verifyTrue(all(isfinite(latestFeature)));
        end
    end
end

function ref = reference(x, y, psi, kappa, vRefBase, refValid)
ref = struct('x_ref', x, 'y_ref', y, 'psi_ref', psi, ...
    'kappa_ref', kappa, 'v_ref_base', vRefBase, 'path_idx', uint32(1), ...
    'ref_valid', refValid);
end

function meas = measurement(x, y, yaw, vx, vy, yawRate, beta, ay, ...
        deltaMeas, deltaRateMeas, axMeas, measValid)
meas = struct('x', x, 'y', y, 'yaw', yaw, 'vx', vx, 'vy', vy, ...
    'yaw_rate', yawRate, 'beta', beta, 'ay', ay, ...
    'delta_meas', deltaMeas, 'ax_meas', axMeas, ...
    'delta_rate_meas', deltaRateMeas, 'meas_valid', measValid);
end

function path = laneChangePath()
x = (0.0:1.0:180.0).';
z1 = 0.096 * (x - 60.0) - 1.2;
z2 = 0.096 * (x - 120.0) - 1.2;
y = laneChangeY(x);
dyDx = 0.1728 * (sech(z1).^2 - sech(z2).^2);
psi = atan(dyDx);
d2yDx2 = -0.0331776 * (sech(z1).^2 .* tanh(z1) ...
    - sech(z2).^2 .* tanh(z2));
kappa = d2yDx2 ./ (1.0 + dyDx.^2).^(3.0 / 2.0);
path = [x, y, psi, kappa, 8.0 * ones(size(x))];
end

function y = laneChangeY(x)
z1 = 0.096 * (x - 60.0) - 1.2;
z2 = 0.096 * (x - 120.0) - 1.2;
y = 1.8 * (1.0 + tanh(z1)) - 1.8 * (1.0 + tanh(z2));
end
