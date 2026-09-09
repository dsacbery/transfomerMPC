classdef TestCarSimStraightSmoke < matlab.unittest.TestCase
    %TestCarSimStraightSmoke Verifies the Day 4 straight-line acceptance gate.

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture(fullfile(projectRoot, ...
                'src')));
        end
    end

    methods (Test)
        function testAcceptsTenSecondStraightLog(testCase)
            log = straightLog();

            summary = tmpsim.verifyCarSimStraightSmoke(log, 0.02, 1e-3);

            testCase.verifyEqual(summary.duration_s, 10.0, AbsTol=1e-12);
            testCase.verifyEqual(summary.sample_count, uint32(501));
        end

        function testRejectsConcurrentThrottleAndBrake(testCase)
            log = straightLog();
            log.brake_cmd(100) = 0.1;

            testCase.verifyError( ...
                @() tmpsim.verifyCarSimStraightSmoke(log, 0.02, 1e-3), ...
                'tmpsim:StraightSmokeActuatorFailure');
        end
    end
end

function log = straightLog()
log.time = (0:0.02:10).';
log.x = 20.0 * log.time;
log.vx = 20.0 * ones(size(log.time));
log.yaw = zeros(size(log.time));
log.delta_cmd = zeros(size(log.time));
log.throttle_cmd = 0.4 * ones(size(log.time));
log.brake_cmd = zeros(size(log.time));
end
