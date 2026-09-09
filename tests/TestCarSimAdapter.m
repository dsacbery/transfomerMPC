classdef TestCarSimAdapter < matlab.unittest.TestCase
    %TestCarSimAdapter Verifies the Day 4 CarSim boundary in isolation.

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
                @() tmpsim.CarSimAdapter(0.01, commandMap()), ...
                'tmpsim:InvalidAdapterSampleTime');
        end

        function testMapsNamedRawMeasurementsToFrozenMeasBus(testCase)
            adapter = tmpsim.CarSimAdapter(0.02, commandMap());
            raw = nominalRawMeasurement();

            meas = adapter.adaptMeasurement(raw);

            testCase.verifyEqual(meas.x, raw.Xo, AbsTol=1e-12);
            testCase.verifyEqual(meas.y, raw.Yo, AbsTol=1e-12);
            testCase.verifyEqual(meas.yaw, deg2rad(raw.Yaw), AbsTol=1e-12);
            testCase.verifyEqual(meas.vx, raw.Vx / 3.6, AbsTol=1e-12);
            testCase.verifyEqual(meas.vy, raw.Vy / 3.6, AbsTol=1e-12);
            testCase.verifyEqual(meas.yaw_rate, deg2rad(raw.AVz), ...
                AbsTol=1e-12);
            testCase.verifyEqual(meas.beta, deg2rad(raw.Beta), ...
                AbsTol=1e-12);
            testCase.verifyEqual(meas.ay, raw.Ay * 9.80665, ...
                AbsTol=1e-12);
            testCase.verifyEqual(meas.delta_meas, ...
                deg2rad((raw.Steer_L1 + raw.Steer_R1) / 2), ...
                AbsTol=1e-12);
            testCase.verifyEqual(meas.ax_meas, raw.Ax * 9.80665, ...
                AbsTol=1e-12);
            testCase.verifyEqual(meas.delta_rate_meas, 0, AbsTol=1e-12);
            testCase.verifyTrue(meas.meas_valid);
        end

        function testRejectsAngularAccelerationAayAsLateralAcceleration(testCase)
            adapter = tmpsim.CarSimAdapter(0.02, commandMap());
            raw = rmfield(nominalRawMeasurement(), 'Ay');
            raw.AAy = 0.25;

            meas = adapter.adaptMeasurement(raw);

            testCase.verifyFalse(meas.meas_valid);
            testCase.verifyEqual(meas.ay, 0, AbsTol=1e-12);
        end

        function testComputesSteeringRateWithMpcSampleTime(testCase)
            adapter = tmpsim.CarSimAdapter(0.02, commandMap());
            firstRaw = nominalRawMeasurement();
            secondRaw = firstRaw;
            secondRaw.Steer_L1 = 5.0;
            secondRaw.Steer_R1 = 4.0;

            firstMeas = adapter.adaptMeasurement(firstRaw);
            secondMeas = adapter.adaptMeasurement(secondRaw);

            expectedRate = (secondMeas.delta_meas - firstMeas.delta_meas) / 0.02;
            testCase.verifyEqual(secondMeas.delta_rate_meas, expectedRate, ...
                AbsTol=1e-12);
        end

        function testRejectsNonfiniteMeasurementsAndResetsDerivative(testCase)
            adapter = tmpsim.CarSimAdapter(0.02, commandMap());
            firstRaw = nominalRawMeasurement();
            invalidRaw = firstRaw;
            invalidRaw.Vy = Inf;
            recoveredRaw = firstRaw;
            recoveredRaw.Steer_L1 = 5.0;
            recoveredRaw.Steer_R1 = 4.0;

            adapter.adaptMeasurement(firstRaw);
            invalidMeas = adapter.adaptMeasurement(invalidRaw);
            recoveredMeas = adapter.adaptMeasurement(recoveredRaw);

            testCase.verifyFalse(invalidMeas.meas_valid);
            testCase.verifyTrue(all(isfinite(struct2array(invalidMeas))));
            testCase.verifyEqual(recoveredMeas.delta_rate_meas, 0, ...
                AbsTol=1e-12);
        end

        function testMapsValidCommandsToCarSimImportChannels(testCase)
            adapter = tmpsim.CarSimAdapter(0.02, commandMap());
            accelerating = command(5.0, 1.25);
            braking = command(-5.0, -3.0);

            acceleratingPlant = adapter.adaptCommand(accelerating);
            brakingPlant = adapter.adaptCommand(braking);

            testCase.verifyEqual(acceleratingPlant.IMP_STEER_L1, 5.0, ...
                AbsTol=1e-12);
            testCase.verifyEqual(acceleratingPlant.IMP_STEER_R1, 5.0, ...
                AbsTol=1e-12);
            testCase.verifyEqual(acceleratingPlant.IMP_THROTTLE_ENGINE, 0.5, ...
                AbsTol=1e-12);
            testCase.verifyEqual(acceleratingPlant.IMP_PCON_BK, 0, ...
                AbsTol=1e-12);
            testCase.verifyEqual(brakingPlant.IMP_THROTTLE_ENGINE, 0, ...
                AbsTol=1e-12);
            testCase.verifyEqual(brakingPlant.IMP_PCON_BK, 5.0, ...
                AbsTol=1e-12);
        end

        function testLimitsInvalidCommandsAndHoldsLastValidCommand(testCase)
            adapter = tmpsim.CarSimAdapter(0.02, commandMap());
            previous = adapter.adaptCommand(command(2.0, 1.0));
            oversized = adapter.adaptCommand(command(60.0, 10.0));
            invalid = command(0.0, NaN);
            heldForInvalid = adapter.adaptCommand(invalid);
            heldByFlag = command(-10.0, -3.0);
            heldByFlag.hold_last_cmd = true;
            heldForFlag = adapter.adaptCommand(heldByFlag);
            heldByInvalidFlag = command(-10.0, -3.0);
            heldByInvalidFlag.cmd_valid = false;
            heldForInvalidFlag = adapter.adaptCommand(heldByInvalidFlag);

            testCase.verifyEqual(oversized.IMP_STEER_L1, 35.0, ...
                AbsTol=1e-12);
            testCase.verifyEqual(oversized.IMP_STEER_R1, 35.0, ...
                AbsTol=1e-12);
            testCase.verifyEqual(oversized.IMP_THROTTLE_ENGINE, 1.0, ...
                AbsTol=1e-12);
            testCase.verifyEqual(oversized.IMP_PCON_BK, 0, AbsTol=1e-12);
            testCase.verifyEqual(heldForInvalid, oversized);
            testCase.verifyEqual(heldForFlag, oversized);
            testCase.verifyEqual(heldForInvalidFlag, oversized);
            testCase.verifyNotEqual(previous, oversized);
            testCase.verifyFalse(oversized.IMP_THROTTLE_ENGINE > 0 ...
                && oversized.IMP_PCON_BK > 0);
        end

        function testRejectsInvalidCommandMap(testCase)
            map = commandMap();
            map.brake_max_mpa = NaN;
            adapter = tmpsim.CarSimAdapter(0.02, map);

            plant = adapter.adaptCommand(command(2.0, 1.0));

            testCase.verifyEqual(plant.IMP_STEER_L1, 0, AbsTol=1e-12);
            testCase.verifyEqual(plant.IMP_STEER_R1, 0, AbsTol=1e-12);
            testCase.verifyEqual(plant.IMP_THROTTLE_ENGINE, 0, AbsTol=1e-12);
            testCase.verifyEqual(plant.IMP_PCON_BK, 0, AbsTol=1e-12);
        end

        function testResetClearsMeasurementAndCommandState(testCase)
            adapter = tmpsim.CarSimAdapter(0.02, commandMap());
            firstRaw = nominalRawMeasurement();
            secondRaw = firstRaw;
            secondRaw.Steer_L1 = 5.0;
            secondRaw.Steer_R1 = 4.0;
            adapter.adaptMeasurement(firstRaw);
            adapter.adaptMeasurement(secondRaw);
            adapter.adaptCommand(command(2.0, 1.0));

            adapter.reset();
            resetMeas = adapter.adaptMeasurement(secondRaw);
            heldAfterReset = command(0.0, 0.0);
            heldAfterReset.hold_last_cmd = true;
            resetPlant = adapter.adaptCommand(heldAfterReset);

            testCase.verifyEqual(resetMeas.delta_rate_meas, 0, AbsTol=1e-12);
            testCase.verifyEqual(resetPlant.IMP_STEER_L1, 0, AbsTol=1e-12);
            testCase.verifyEqual(resetPlant.IMP_STEER_R1, 0, AbsTol=1e-12);
            testCase.verifyEqual(resetPlant.IMP_THROTTLE_ENGINE, 0, AbsTol=1e-12);
            testCase.verifyEqual(resetPlant.IMP_PCON_BK, 0, AbsTol=1e-12);
        end
    end
end

function raw = nominalRawMeasurement()
raw.Xo = 12.5;
raw.Yo = -1.25;
raw.Yaw = 15.0;
raw.Vx = 72.0;
raw.Vy = -3.6;
raw.AVz = 20.0;
raw.Beta = -2.0;
raw.Ay = 0.4;
raw.Steer_L1 = 3.0;
raw.Steer_R1 = 2.0;
raw.Ax = -0.2;
end

function map = commandMap()
map.delta_min_rad = deg2rad(-35);
map.delta_max_rad = deg2rad(35);
map.ax_min_mps2 = -6.0;
map.ax_max_mps2 = 2.5;
map.throttle_max = 1.0;
map.brake_max_mpa = 10.0;
end

function cmd = command(deltaDeg, axCmd)
cmd.delta_cmd = deg2rad(deltaDeg);
cmd.ax_cmd = axCmd;
cmd.cmd_valid = true;
cmd.hold_last_cmd = false;
cmd.control_mode_id = uint8(1);
end
