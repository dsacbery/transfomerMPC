classdef TestHistoryBuffer < matlab.unittest.TestCase
    %TESTHISTORYBUFFER Verifies the frozen chronological HistoryBus contract.

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture(fullfile(projectRoot, ...
                'src')));
        end
    end

    methods (Test)
        function testResetOutputsAnEmptyFrozenHistoryBus(testCase)
            buffer = tmpsim.HistoryBuffer(0.02);
            latestFeature = featureRow(100.0);

            history = buffer.step(latestFeature, true);

            testCase.verifyEqual(fieldnames(history).', { ...
                'window', 'latest_feature', 'window_ready', 'history_len', ...
                'sample_time'});
            testCase.verifySize(history.window, [16 14]);
            testCase.verifyClass(history.window, 'double');
            testCase.verifyEqual(history.window, zeros(16, 14), AbsTol=1e-12);
            testCase.verifyEqual(history.latest_feature, latestFeature, AbsTol=1e-12);
            testCase.verifySize(history.latest_feature, [1 14]);
            testCase.verifyClass(history.latest_feature, 'double');
            testCase.verifyTrue(all(isfinite(history.latest_feature)));
            testCase.verifyFalse(history.window_ready);
            testCase.verifyClass(history.window_ready, 'logical');
            testCase.verifyEqual(history.history_len, uint16(0));
            testCase.verifyClass(history.history_len, 'uint16');
            testCase.verifyEqual(history.sample_time, 0.02, AbsTol=1e-12);
            testCase.verifyClass(history.sample_time, 'double');
        end

        function testWritesFirstFeatureAndZeroFillsRemainingRows(testCase)
            buffer = tmpsim.HistoryBuffer(0.02);
            latestFeature = featureRow(1.0);

            history = buffer.step(latestFeature, false);

            testCase.verifyEqual(history.window(1, :), latestFeature, AbsTol=1e-12);
            testCase.verifyEqual(history.window(2:end, :), zeros(15, 14), ...
                AbsTol=1e-12);
            testCase.verifyEqual(history.latest_feature, latestFeature, AbsTol=1e-12);
            testCase.verifyFalse(history.window_ready);
            testCase.verifyEqual(history.history_len, uint16(1));
        end

        function testPreservesPartialHistoryInOldToNewOrder(testCase)
            buffer = tmpsim.HistoryBuffer(0.02);
            features = [featureRow(1.0); featureRow(20.0); featureRow(40.0)];

            for index = 1:size(features, 1)
                history = buffer.step(features(index, :), false);
            end

            testCase.verifyEqual(history.window(1:3, :), features, AbsTol=1e-12);
            testCase.verifyEqual(history.window(4:end, :), zeros(13, 14), ...
                AbsTol=1e-12);
            testCase.verifyFalse(history.window_ready);
            testCase.verifyEqual(history.history_len, uint16(3));
        end

        function testMarksWindowReadyWhenExactlySixteenRowsHaveBeenWritten(testCase)
            buffer = tmpsim.HistoryBuffer(0.02);
            features = featureRows(1.0, 16);

            for index = 1:16
                history = buffer.step(features(index, :), false);
            end

            testCase.verifyEqual(history.window, features, AbsTol=1e-12);
            testCase.verifyTrue(history.window_ready);
            testCase.verifyEqual(history.history_len, uint16(16));
            testCase.verifyEqual(history.latest_feature, features(end, :), ...
                AbsTol=1e-12);
        end

        function testSeventeenthWriteOverwritesOnlyTheOldestFeature(testCase)
            buffer = tmpsim.HistoryBuffer(0.02);
            features = featureRows(1.0, 17);

            for index = 1:17
                history = buffer.step(features(index, :), false);
            end

            testCase.verifyEqual(history.window, features(2:end, :), AbsTol=1e-12);
            testCase.verifyTrue(history.window_ready);
            testCase.verifyEqual(history.history_len, uint16(16));
        end

        function testAlwaysReturnsFullWindowInChronologicalOrderAfterWraps(testCase)
            buffer = tmpsim.HistoryBuffer(0.02);
            features = featureRows(10.0, 33);

            for index = 1:size(features, 1)
                history = buffer.step(features(index, :), false);
            end

            testCase.verifyEqual(history.window, features(end - 15:end, :), ...
                AbsTol=1e-12);
            testCase.verifyEqual(history.latest_feature, features(end, :), ...
                AbsTol=1e-12);
            testCase.verifyTrue(history.window_ready);
            testCase.verifyEqual(history.history_len, uint16(16));
        end

        function testResetClearsWindowAndStateAfterPriorWrites(testCase)
            buffer = tmpsim.HistoryBuffer(0.02);
            for index = 1:4
                buffer.step(featureRow(double(index)), false);
            end
            resetFeature = featureRow(80.0);

            history = buffer.step(resetFeature, true);
            next = buffer.step(featureRow(90.0), false);

            testCase.verifyEqual(history.window, zeros(16, 14), AbsTol=1e-12);
            testCase.verifyEqual(history.latest_feature, resetFeature, AbsTol=1e-12);
            testCase.verifyFalse(history.window_ready);
            testCase.verifyEqual(history.history_len, uint16(0));
            testCase.verifyEqual(next.window(1, :), featureRow(90.0), ...
                AbsTol=1e-12);
            testCase.verifyEqual(next.window(2:end, :), zeros(15, 14), ...
                AbsTol=1e-12);
            testCase.verifyEqual(next.history_len, uint16(1));
        end

        function testRejectsNonfiniteAndWrongSizedFeaturesWithoutChangingState(testCase)
            buffer = tmpsim.HistoryBuffer(0.02);
            first = featureRow(1.0);
            second = featureRow(20.0);
            invalidNonfinite = featureRow(40.0);
            invalidNonfinite(7) = NaN;

            buffer.step(first, false);
            testCase.verifyError( ...
                @() buffer.step(invalidNonfinite, false), ...
                'tmpsim:InvalidHistoryFeature');
            testCase.verifyError( ...
                @() buffer.step(zeros(14, 1), false), ...
                'tmpsim:InvalidHistoryFeature');
            testCase.verifyError( ...
                @() buffer.step(zeros(1, 13), false), ...
                'tmpsim:InvalidHistoryFeature');

            history = buffer.step(second, false);

            testCase.verifyEqual(history.window(1:2, :), [first; second], ...
                AbsTol=1e-12);
            testCase.verifyEqual(history.window(3:end, :), zeros(14, 14), ...
                AbsTol=1e-12);
            testCase.verifyEqual(history.history_len, uint16(2));
        end

        function testFeedsLatestErrorFeatureIntoHistoryBus(testCase)
            builder = tmpsim.ErrorFeatureBuilder(0.02);
            buffer = tmpsim.HistoryBuffer(0.02);
            ref = struct( ...
                'x_ref', 10.0, ...
                'y_ref', 0.0, ...
                'psi_ref', 0.0, ...
                'kappa_ref', 0.0, ...
                'v_ref_base', 8.0, ...
                'path_idx', uint32(1), ...
                'ref_valid', true);
            meas = struct( ...
                'x', 10.0, ...
                'y', 0.0, ...
                'yaw', 0.0, ...
                'vx', 8.0, ...
                'vy', 0.0, ...
                'yaw_rate', 0.0, ...
                'beta', 0.0, ...
                'ay', 0.1, ...
                'delta_meas', 0.2, ...
                'delta_rate_meas', 0.3, ...
                'ax_meas', 0.4, ...
                'meas_valid', true);

            [~, latestFeature] = builder.build(ref, meas, false);
            history = buffer.step(latestFeature, false);

            testCase.verifyEqual(history.latest_feature, latestFeature, ...
                AbsTol=1e-12);
            testCase.verifyEqual(history.window(1, :), latestFeature, ...
                AbsTol=1e-12);
            testCase.verifySize(history.window, [16 14]);
            testCase.verifyTrue(all(isfinite(history.window(:))));
            testCase.verifyEqual(history.sample_time, 0.02, AbsTol=1e-12);
        end
    end
end

function row = featureRow(offset)
row = double(offset + (0:13));
end

function rows = featureRows(firstOffset, count)
rows = zeros(count, 14);
for index = 1:count
    rows(index, :) = featureRow(firstOffset + 20.0 * double(index - 1));
end
end
