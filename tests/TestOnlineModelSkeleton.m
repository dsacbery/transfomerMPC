classdef TestOnlineModelSkeleton < matlab.unittest.TestCase
    %TestOnlineModelSkeleton Verifies the Day 3 mock-only top-level model.

    properties (Constant)
        ModelName = 'tmpsim_online'
    end

    properties
        ProjectRoot
        ModelPath
        DictionaryPath
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            testCase.ProjectRoot = fileparts(fileparts(mfilename('fullpath')));
            testCase.ModelPath = fullfile(testCase.ProjectRoot, 'model', ...
                [testCase.ModelName '.slx']);
            testCase.DictionaryPath = fullfile(testCase.ProjectRoot, ...
                'model', 'tmpsim_online.sldd');
            addpath(fullfile(testCase.ProjectRoot, 'scripts'));
            addpath(fullfile(testCase.ProjectRoot, 'config'));
            addpath(fullfile(testCase.ProjectRoot, 'src'));
        end
    end

    methods (TestMethodTeardown)
        function closeModel(~)
            if bdIsLoaded('tmpsim_online')
                close_system('tmpsim_online', 0);
            end
        end
    end

    methods (Test)
        function testBuilderCreatesDictionaryAttachedModel(testCase)
            testCase.assertEqual(exist('create_tmpsim_online_model', ...
                'file'), 2);
            create_tmpsim_online_model();

            testCase.verifyTrue(isfile(testCase.ModelPath));
            load_system(testCase.ModelPath);
            testCase.verifyEqual(string(get_param(testCase.ModelName, ...
                'DataDictionary')), "tmpsim_online.sldd");
        end

        function testBuilderPreservesFrozenDictionary(testCase)
            dictionaryBefore = readFileBytes(testCase.DictionaryPath);

            create_tmpsim_online_model();

            dictionaryAfter = readFileBytes(testCase.DictionaryPath);
            testCase.verifyEqual(dictionaryAfter, dictionaryBefore);
        end

        function testTopLevelContainsRequiredSubsystems(testCase)
            create_tmpsim_online_model();
            load_system(testCase.ModelPath);

            expected = ["ScenarioManager", "CarSimAdapter", ...
                "ErrorFeatureBuilder", "HistoryBuffer", ...
                "TransformerRisk", "RiskSupervisor", ...
                "LateralControllerVariant", "SpeedOuterLoop", "Logger"];
            subsystemPaths = find_system(testCase.ModelName, ...
                'SearchDepth', 1, 'BlockType', 'SubSystem');
            actual = string(get_param(subsystemPaths, 'Name'));

            testCase.verifyEqual(sort(actual(:)), sort(expected(:)));
        end

        function testModelCompilesAtFrozenRates(testCase)
            create_tmpsim_online_model();
            addpath(fullfile(testCase.ProjectRoot, 'model'));
            load_system(testCase.ModelPath);

            testCase.verifyWarningFree(@() set_param(testCase.ModelName, ...
                'SimulationCommand', 'update'));
            testCase.verifyEqual(get_param(testCase.ModelName, 'FixedStep'), ...
                '0.02');
            sampleTimes = get_param({ ...
                'tmpsim_online/ScenarioManager/RefBus_x_ref', ...
                'tmpsim_online/TransformerRisk/RiskRawBus_r_low_raw'}, ...
                'SampleTime');
            testCase.verifyEqual(string(sampleTimes(:)), ["0.02"; "0.1"]);
        end

        function testBusElementsDoNotOwnSampleTimes(testCase)
            dictionary = Simulink.data.dictionary.open(testCase.DictionaryPath);
            testCase.addTeardown(@() close(dictionary));
            entries = find(getSection(dictionary, 'Design Data'));

            for entryIndex = 1:numel(entries)
                bus = getValue(entries(entryIndex));
                for elementIndex = 1:numel(bus.Elements)
                    testCase.verifyEqual(bus.Elements(elementIndex).SampleTime, ...
                        -1);
                end
            end
        end

        function testShortMockSimulationCompletes(testCase)
            create_tmpsim_online_model();
            addpath(fullfile(testCase.ProjectRoot, 'model'));

            out = sim(testCase.ModelPath, 'StopTime', '0.20');

            testCase.verifyGreaterThanOrEqual( ...
                out.SimulationMetadata.ModelInfo.StopTime, 0.20);
        end
    end
end

function bytes = readFileBytes(filePath)
fileId = fopen(filePath, 'r');
cleanupFile = onCleanup(@() fclose(fileId));
bytes = fread(fileId, Inf, '*uint8');
clear cleanupFile
end
