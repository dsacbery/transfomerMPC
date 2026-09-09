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
    end
end
