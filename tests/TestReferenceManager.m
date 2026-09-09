classdef TestReferenceManager < matlab.unittest.TestCase
    %TestReferenceManager Verifies caller-supplied reference path lookup.

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture(fullfile(projectRoot, ...
                'src')));
        end
    end

    methods (Test)
        function testSearchesMonotonicallyForwardOnStraightPath(testCase)
            manager = tmpsim.ReferenceManager(straightPath(), 10.0, 1.0);

            first = manager.step(measurement(2.1, 0.0), false);
            later = manager.step(measurement(1.1, 0.0), false);

            testCase.verifyEqual(first.path_idx, uint32(3));
            testCase.verifyEqual(later.path_idx, first.path_idx);
            testCase.verifyEqual(later.x_ref, 2.0, AbsTol=1e-12);
            testCase.verifyTrue(later.ref_valid);
        end

        function testLimitsEachForwardSearchToFiftyPoints(testCase)
            path = straightPath();
            path = [path; (11:100).', zeros(90, 3), 6.0 * ones(90, 1)];
            manager = tmpsim.ReferenceManager(path, 10.0, 1.0);

            ref = manager.step(measurement(100.0, 0.0), false);

            testCase.verifyEqual(ref.path_idx, uint32(50));
            testCase.verifyEqual(ref.x_ref, 49.0, AbsTol=1e-12);
        end

        function testSelectsNearestPointForLargeFiniteCoordinates(testCase)
            path = [0.0, 0.0, 0.0, 0.0, 6.0; ...
                1.5e154, 0.0, 0.0, 0.0, 6.0];
            manager = tmpsim.ReferenceManager(path, 10.0, 1.0);

            ref = manager.step(measurement(3.0e154, 0.0), false);

            testCase.verifyEqual(ref.path_idx, uint32(2));
            testCase.verifyFalse(ref.ref_valid);
        end

        function testLocksAtTerminalPointOnLeftTurnArc(testCase)
            path = leftTurnArcPath();
            manager = tmpsim.ReferenceManager(path, 10.0, 1.0);

            terminal = manager.step(measurement(1000.0, 1000.0), false);
            held = manager.step(measurement(-1000.0, -1000.0), false);

            testCase.verifyEqual(terminal.x_ref, path(end, 1), AbsTol=1e-12);
            testCase.verifyEqual(terminal.y_ref, path(end, 2), AbsTol=1e-12);
            testCase.verifyEqual(terminal.psi_ref, path(end, 3), AbsTol=1e-12);
            testCase.verifyEqual(terminal.kappa_ref, path(end, 4), AbsTol=1e-12);
            testCase.verifyGreaterThan(terminal.kappa_ref, 0.0);
            testCase.verifyEqual(terminal.path_idx, uint32(size(path, 1)));
            testCase.verifyFalse(terminal.ref_valid);
            testCase.verifyEqual(held.path_idx, terminal.path_idx);
            testCase.verifyFalse(held.ref_valid);
        end

        function testResetReturnsFirstLaneChangePointWithoutAdvancing(testCase)
            path = tanhLaneChangePath();
            manager = tmpsim.ReferenceManager(path, 10.0, 1.0);

            advanced = manager.step(measurement(30.0, 3.5), false);
            resetReference = manager.step(measurement(30.0, 3.5), true);

            testCase.verifyGreaterThan(advanced.path_idx, uint32(1));
            testCase.verifyEqual(resetReference.path_idx, uint32(1));
            testCase.verifyEqual(resetReference.x_ref, path(1, 1), AbsTol=1e-12);
            testCase.verifyEqual(resetReference.y_ref, path(1, 2), AbsTol=1e-12);
            testCase.verifyEqual(resetReference.psi_ref, path(1, 3), AbsTol=1e-12);
            testCase.verifyTrue(resetReference.ref_valid);
        end

        function testClipsBaseSpeedToExplicitBounds(testCase)
            path = straightPath();
            path(:, 5) = 6.0;
            path(1, 5) = 0.25;
            path(2, 5) = 16.0;
            manager = tmpsim.ReferenceManager(path, 10.0, 1.0);

            slow = manager.step(measurement(0.0, 0.0), false);
            fast = manager.step(measurement(1.0, 0.0), false);

            testCase.verifyEqual(slow.v_ref_base, 1.0, AbsTol=1e-12);
            testCase.verifyEqual(fast.v_ref_base, 10.0, AbsTol=1e-12);
        end

        function testRejectsInvalidPathAndSpeedBounds(testCase)
            invalidPath = straightPath();
            invalidPath(2, 1) = NaN;

            testCase.verifyError( ...
                @() tmpsim.ReferenceManager(invalidPath, 10.0, 1.0), ...
                'tmpsim:InvalidReferencePath');
            testCase.verifyError( ...
                @() tmpsim.ReferenceManager(straightPath(), Inf, 1.0), ...
                'tmpsim:InvalidReferenceSpeedBounds');
            testCase.verifyError( ...
                @() tmpsim.ReferenceManager(straightPath(), 1.0, 2.0), ...
                'tmpsim:InvalidReferenceSpeedBounds');
        end

        function testInvalidMeasurementHoldsCurrentIndex(testCase)
            manager = tmpsim.ReferenceManager(straightPath(), 10.0, 1.0);
            current = manager.step(measurement(3.0, 0.0), false);
            invalid = measurement(NaN, 0.0);

            held = manager.step(invalid, false);

            testCase.verifyEqual(held.path_idx, current.path_idx);
            testCase.verifyEqual(held.x_ref, current.x_ref, AbsTol=1e-12);
            testCase.verifyTrue(held.ref_valid);
        end

        function testFalseMeasurementValidityHoldsCurrentIndex(testCase)
            manager = tmpsim.ReferenceManager(straightPath(), 10.0, 1.0);
            current = manager.step(measurement(3.0, 0.0), false);
            invalid = measurement(100.0, 0.0);
            invalid.meas_valid = false;

            held = manager.step(invalid, false);

            testCase.verifyEqual(held.path_idx, current.path_idx);
            testCase.verifyEqual(held.x_ref, current.x_ref, AbsTol=1e-12);
            testCase.verifyTrue(held.ref_valid);
        end

        function testNonlogicalMeasurementValidityHoldsCurrentIndex(testCase)
            manager = tmpsim.ReferenceManager(straightPath(), 10.0, 1.0);
            current = manager.step(measurement(3.0, 0.0), false);
            invalid = measurement(100.0, 0.0);
            invalid.meas_valid = 1;

            held = manager.step(invalid, false);

            testCase.verifyEqual(held.path_idx, current.path_idx);
            testCase.verifyEqual(held.x_ref, current.x_ref, AbsTol=1e-12);
            testCase.verifyTrue(held.ref_valid);
        end

        function testMissingMeasurementValidityHoldsCurrentIndex(testCase)
            manager = tmpsim.ReferenceManager(straightPath(), 10.0, 1.0);
            current = manager.step(measurement(3.0, 0.0), false);
            missingValidity = struct('x', 100.0, 'y', 0.0);

            held = manager.step(missingValidity, false);

            testCase.verifyEqual(held.path_idx, current.path_idx);
            testCase.verifyEqual(held.x_ref, current.x_ref, AbsTol=1e-12);
            testCase.verifyTrue(held.ref_valid);
        end

        function testReturnsFrozenRefBusFields(testCase)
            manager = tmpsim.ReferenceManager(straightPath(), 10.0, 1.0);

            ref = manager.step(measurement(0.0, 0.0), false);

            testCase.verifyEqual(fieldnames(ref).', { ...
                'x_ref', 'y_ref', 'psi_ref', 'kappa_ref', 'v_ref_base', ...
                'path_idx', 'ref_valid'});
            testCase.verifyClass(ref.path_idx, 'uint32');
            testCase.verifyClass(ref.ref_valid, 'logical');
        end
    end
end

function path = straightPath()
x = (0:10).';
path = [x, zeros(size(x)), zeros(size(x)), zeros(size(x)), ...
    6.0 * ones(size(x))];
end

function path = leftTurnArcPath()
radius = 20.0;
theta = linspace(0.0, pi / 2, 31).';
x = radius * sin(theta);
y = radius * (1.0 - cos(theta));
path = [x, y, theta, repmat(1.0 / radius, size(theta)), ...
    8.0 * ones(size(theta))];
end

function path = tanhLaneChangePath()
x = linspace(0.0, 40.0, 81).';
laneWidth = 3.5;
centerX = 20.0;
transitionLength = 5.0;
z = (x - centerX) / transitionLength;

% Analytic test fixture only: y = d/2 * (1 + tanh((x - xc) / L)).
y = laneWidth / 2.0 * (1.0 + tanh(z));
dyDx = laneWidth / (2.0 * transitionLength) * sech(z).^2;
d2yDx2 = -laneWidth / transitionLength^2 * sech(z).^2 .* tanh(z);
psi = atan(dyDx);
kappa = d2yDx2 ./ (1.0 + dyDx.^2).^(3.0 / 2.0);
path = [x, y, psi, kappa, 8.0 * ones(size(x))];
end

function meas = measurement(x, y)
meas = struct('x', x, 'y', y, 'meas_valid', true);
end
