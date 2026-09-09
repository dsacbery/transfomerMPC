# Day 3 Online Skeleton Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (- [ ]) syntax for tracking.

**Goal:** Build and verify model/tmpsim_online.slx as a reproducible typed-bus, mock-only Simulink top-level skeleton.

**Architecture:** scripts/create_tmpsim_online_model.m recreates the model, attaches model/tmpsim_online.sldd, creates the nine required top-level subsystems, and wires their Day 2 bus contracts through finite mock values. All subsystems use standard blocks only: Bus Creator, Constant, Inport, Outport, Unit Delay, and Terminator. tests/TestOnlineModelSkeleton.m compiles and simulates the resulting model.

**Tech Stack:** MATLAB R2024b, Simulink, Simulink Data Dictionary, matlab.unittest, Git.

---

## File Structure

- Create: scripts/create_tmpsim_online_model.m - deterministic model builder.
- Create: tests/TestOnlineModelSkeleton.m - topology, dictionary, timing, compile, and smoke tests.
- Create: model/tmpsim_online.slx - generated model committed after verification.
- Create: docs/Day_03_Handoff.md - exact commands, evidence, scope boundary, next work.
- Modify: model/README.md - explain regeneration.

Day 2 files remain unmodified: config/tmpsim_config.m, scripts/create_tmpsim_dictionary.m, model/tmpsim_online.sldd, and tests/TestSignalContracts.m.

### Task 1: Write The Failing Model Contract Test

**Files:**

- Create: tests/TestOnlineModelSkeleton.m
- Reference: tests/TestSignalContracts.m
- Reference: config/tmpsim_config.m

- [ ] **Step 1: Add the initial test class**

Create the following test before writing a model builder.

~~~matlab
classdef TestOnlineModelSkeleton < matlab.unittest.TestCase
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
                'DataDictionary')), string(testCase.DictionaryPath));
        end
    end
end
~~~

- [ ] **Step 2: Verify the red state**

Run:

~~~powershell
& 'G:\Matlab2024b\bin\matlab.exe' -batch "cd('D:\Desktop\TransformerMPC'); results=runtests('tests/TestOnlineModelSkeleton.m','Name','testBuilderCreatesDictionaryAttachedModel'); assertSuccess(results)"
~~~

Expected: one failure because create_tmpsim_online_model is absent. The failure must be the assertion for the missing builder, not a test syntax/API error.

- [ ] **Step 3: Commit the red test**

~~~powershell
git add tests/TestOnlineModelSkeleton.m
git commit -m "test: define day 3 model skeleton contract"
~~~

### Task 2: Build The Deterministic Skeleton

**Files:**

- Create: scripts/create_tmpsim_online_model.m
- Create by generation: model/tmpsim_online.slx
- Test: tests/TestOnlineModelSkeleton.m

- [ ] **Step 1: Implement the model builder entry point**

Create the builder with this exact control flow.

~~~matlab
function modelPath = create_tmpsim_online_model()
%CREATE_TMPSIM_ONLINE_MODEL Creates the Day 3 mock-only online skeleton.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'scripts'));
addpath(fullfile(projectRoot, 'src'));

create_tmpsim_dictionary();
cfg = tmpsim_config();
tmpsim.validateOnlineConfig(cfg);

modelName = 'tmpsim_online';
modelPath = fullfile(projectRoot, 'model', [modelName '.slx']);
dictionaryPath = fullfile(projectRoot, 'model', 'tmpsim_online.sldd');
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if isfile(modelPath)
    delete(modelPath);
end

new_system(modelName);
set_param(modelName, ...
    'DataDictionary', dictionaryPath, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(cfg.sample.Ts_mpc), ...
    'StopTime', '0.20', ...
    'SignalLogging', 'on', ...
    'SignalLoggingName', 'logsout', ...
    'ReturnWorkspaceOutputs', 'on');

buildScenarioManager(modelName, cfg);
buildCarSimAdapter(modelName, cfg);
buildErrorFeatureBuilder(modelName, cfg);
buildHistoryBuffer(modelName, cfg);
buildTransformerRisk(modelName, cfg);
buildRiskSupervisor(modelName, cfg);
buildLateralControllerVariant(modelName, cfg);
buildSpeedOuterLoop(modelName, cfg);
buildLogger(modelName, cfg);
buildCommandAssembler(modelName, cfg);
wireTopLevel(modelName);

set_param(modelName, 'SimulationCommand', 'update');
save_system(modelName, modelPath);
close_system(modelName, 0);
end
~~~

Define all remaining local helpers in this same file. No MATLAB Function, System, S-Function, CarSim, MPC, or neural-network block is permitted.

- [ ] **Step 2: Implement type-safe mock source helpers**

Use a Bus Creator with the Day 2 bus object, a Constant for every bus element, and a typed Outport. The concrete helper API is:

~~~matlab
function addBusSource(subsystem, busName, elementNames, values, types, ...
        sampleTime, outputName)
creatorPath = subsystem + "/" + busName + " Mock";
add_block('simulink/Signal Routing/Bus Creator', creatorPath, ...
    'Inputs', num2str(numel(elementNames)), ...
    'UseBusObject', 'on', 'BusObject', busName, ...
    'Position', [230 35 260 55]);
for index = 1:numel(elementNames)
    addConstant(subsystem, elementNames(index), values{index}, ...
        types(index), sampleTime, ...
        [35 25 + 45*(index-1) 135 45 + 45*(index-1)]);
    add_line(subsystem, elementNames(index) + "/1", ...
        busName + " Mock/" + string(index), 'autorouting', 'on');
end
addBusOutput(subsystem, outputName, busName, sampleTime, [330 35 360 55]);
add_line(subsystem, busName + " Mock/1", outputName + "/1", ...
    'autorouting', 'on');
end

function addConstant(subsystem, name, value, type, sampleTime, position)
add_block('simulink/Sources/Constant', subsystem + "/" + name, ...
    'Value', value, 'OutDataTypeStr', type, ...
    'SampleTime', num2str(sampleTime), 'Position', position);
end

function addBusOutput(subsystem, name, busName, sampleTime, position)
add_block('simulink/Sinks/Out1', subsystem + "/" + name, ...
    'OutDataTypeStr', "Bus: " + busName, ...
    'SampleTime', num2str(sampleTime), 'Position', position);
end
~~~

For non-scalar continuous elements, use exact fixed shapes: zeros(16,14) for HistoryBus.window and zeros(1,14) for HistoryBus.latest_feature/latest_feature output. Use false for boolean fields, uint8(0) for statuses/modes, uint16(0) and uint32(0) for counters/indexes. Numeric fields are finite doubles. Do not set any property on Simulink.BusElement.

- [ ] **Step 3: Implement the required subsystem interfaces**

Create the nine top-level subsystem blocks at non-overlapping positions. All source blocks and ports use cfg.sample.Ts_mpc except TransformerRisk's RiskRawBus mock and Unit Delay, which use cfg.sample.Ts_tr. Inputs are connected internally to Terminator blocks. Include a Bus Unit Delay between the Bus Creator mock source and the Outport inside HistoryBuffer, TransformerRisk, and CarSimAdapter.

| Subsystem | Inputs | Outputs |
| --- | --- | --- |
| ScenarioManager | none | ref_bus: RefBus |
| CarSimAdapter | cmd_bus: CmdBus | meas_bus: MeasBus |
| ErrorFeatureBuilder | ref_bus: RefBus, meas_bus: MeasBus | err_bus: ErrBus, latest_feature: double[1x14] |
| HistoryBuffer | latest_feature: double[1x14] | history_bus: HistoryBus |
| TransformerRisk | history_bus: HistoryBus | risk_raw_bus: RiskRawBus |
| RiskSupervisor | risk_raw_bus: RiskRawBus, err_bus: ErrBus, meas_bus: MeasBus, ref_bus: RefBus | risk_bus: RiskBus, mpc_param_bus: MPCParamBus, v_ref_adapt: double |
| LateralControllerVariant | err_bus: ErrBus, meas_bus: MeasBus, ref_bus: RefBus, risk_bus: RiskBus, mpc_param_bus: MPCParamBus | delta_cmd: double, mpc_feasible: boolean, mpc_status_code: uint8, solve_time_ms: double |
| SpeedOuterLoop | meas_bus: MeasBus, v_ref_adapt: double | ax_cmd: double, speed_status_code: uint8 |
| Logger | ref_bus: RefBus, meas_bus: MeasBus, err_bus: ErrBus, history_bus: HistoryBus, risk_raw_bus: RiskRawBus, risk_bus: RiskBus, mpc_param_bus: MPCParamBus, cmd_bus: CmdBus | none |

Set mock outputs as follows: measurement/reference/error/history values are zero and their valid flags false; TransformerRisk has k_v_raw=1.0, risk_raw_valid=false, risk_source_id=uint8(0), transformer_status_code=uint8(1); RiskSupervisor has k_v=1.0, risk_valid=true, degraded_mode=false, risk_source_id=uint8(0), plus all finite MPC baseline values from cfg; lateral and speed commands are zero with mpc_feasible=false, mpc_status_code=uint8(0), speed_status_code=uint8(0). These are fixed mock contracts, not algorithms.

- [ ] **Step 4: Build CommandAssembler and wire the top level**

Create a top-level Bus Creator named CommandAssembler for CmdBus. Its inputs are LateralControllerVariant.delta_cmd, SpeedOuterLoop.ax_cmd, and Constants true, false, uint8(1) for cmd_valid, hold_last_cmd, control_mode_id. Its sample time is cfg.sample.Ts_mpc and output signal is named cmd_bus.

Connect exact source and destination ports:

~~~matlab
connections = [ ...
    "ScenarioManager/1", "ErrorFeatureBuilder/1"; ...
    "ScenarioManager/1", "RiskSupervisor/4"; ...
    "CarSimAdapter/1", "ErrorFeatureBuilder/2"; ...
    "CarSimAdapter/1", "RiskSupervisor/3"; ...
    "CarSimAdapter/1", "LateralControllerVariant/2"; ...
    "CarSimAdapter/1", "SpeedOuterLoop/1"; ...
    "ErrorFeatureBuilder/1", "RiskSupervisor/2"; ...
    "ErrorFeatureBuilder/1", "LateralControllerVariant/1"; ...
    "ErrorFeatureBuilder/2", "HistoryBuffer/1"; ...
    "HistoryBuffer/1", "TransformerRisk/1"; ...
    "HistoryBuffer/1", "Logger/4"; ...
    "TransformerRisk/1", "RiskSupervisor/1"; ...
    "RiskSupervisor/1", "LateralControllerVariant/4"; ...
    "RiskSupervisor/2", "LateralControllerVariant/5"; ...
    "RiskSupervisor/3", "SpeedOuterLoop/2"; ...
    "LateralControllerVariant/1", "CommandAssembler/1"; ...
    "SpeedOuterLoop/1", "CommandAssembler/2"; ...
    "CommandAssembler/1", "CarSimAdapter/1"; ...
    "ScenarioManager/1", "Logger/1"; ...
    "CarSimAdapter/1", "Logger/2"; ...
    "ErrorFeatureBuilder/1", "Logger/3"; ...
    "TransformerRisk/1", "Logger/5"; ...
    "RiskSupervisor/1", "Logger/6"; ...
    "RiskSupervisor/2", "Logger/7"; ...
    "CommandAssembler/1", "Logger/8"];
for row = 1:size(connections, 1)
    add_line(modelName, connections(row, 1), connections(row, 2), ...
        'autorouting', 'on');
end
~~~

- [ ] **Step 5: Verify the original test is green**

Run:

~~~powershell
& 'G:\Matlab2024b\bin\matlab.exe' -batch "cd('D:\Desktop\TransformerMPC'); results=runtests('tests/TestOnlineModelSkeleton.m','Name','testBuilderCreatesDictionaryAttachedModel'); assertSuccess(results)"
~~~

Expected: one passing test. The generated model has the absolute DataDictionary property for model/tmpsim_online.sldd.

- [ ] **Step 6: Commit the working builder and generated model**

~~~powershell
git add scripts/create_tmpsim_online_model.m tests/TestOnlineModelSkeleton.m model/tmpsim_online.slx
git commit -m "feat: add day 3 Simulink model skeleton"
~~~

### Task 3: Extend Tests First For Topology, Timing, Compilation, And Simulation

**Files:**

- Modify: tests/TestOnlineModelSkeleton.m
- Test: tests/TestOnlineModelSkeleton.m

- [ ] **Step 1: Add failing requirements tests**

Add these test methods before changing the builder:

~~~matlab
function testTopLevelContainsRequiredSubsystems(testCase)
    create_tmpsim_online_model();
    load_system(testCase.ModelPath);
    expected = ["ScenarioManager", "CarSimAdapter", "ErrorFeatureBuilder", ...
        "HistoryBuffer", "TransformerRisk", "RiskSupervisor", ...
        "LateralControllerVariant", "SpeedOuterLoop", "Logger"];
    actual = string(get_param(find_system(testCase.ModelName, ...
        'SearchDepth', 1, 'BlockType', 'SubSystem'), 'Name'));
    testCase.verifyEqual(sort(actual), sort(expected));
end

function testModelCompilesAtFrozenRates(testCase)
    create_tmpsim_online_model();
    load_system(testCase.ModelPath);
    testCase.verifyWarningFree(@() set_param(testCase.ModelName, ...
        'SimulationCommand', 'update'));
    testCase.verifyEqual(get_param(testCase.ModelName, 'FixedStep'), '0.02');
    sampleTimes = get_param([ ...
        'tmpsim_online/ScenarioManager/x_ref', ...
        'tmpsim_online/TransformerRisk/r_low_raw'], 'SampleTime');
    testCase.verifyEqual(string(sampleTimes), ["0.02"; "0.1"]);
end

function testBusElementsDoNotOwnSampleTimes(testCase)
    dictionary = Simulink.data.dictionary.open(testCase.DictionaryPath);
    testCase.addTeardown(@() close(dictionary));
    entries = find(getSection(dictionary, 'Design Data'));
    for index = 1:numel(entries)
        bus = getValue(entries(index));
        for elementIndex = 1:numel(bus.Elements)
            testCase.verifyFalse(isprop(bus.Elements(elementIndex), ...
                'SampleTime'));
        end
    end
end

function testShortMockSimulationCompletes(testCase)
    create_tmpsim_online_model();
    out = sim(testCase.ModelPath, 'StopTime', '0.20');
    testCase.verifyGreaterThanOrEqual( ...
        out.SimulationMetadata.ModelInfo.StopTime, 0.20);
end
~~~

- [ ] **Step 2: Verify the expected red state**

Run:

~~~powershell
& 'G:\Matlab2024b\bin\matlab.exe' -batch "cd('D:\Desktop\TransformerMPC'); results=runtests('tests/TestOnlineModelSkeleton.m'); assertSuccess(results)"
~~~

Expected: failure only for an actual unimplemented tested condition or a Simulink API mismatch. Correct test API usage if R2024b proves a parameter is wrong; preserve semantic checks for nine subsystems, 0.02/0.10 timing ownership, no BusElement sampling, compile, and a 0.2-second simulation.

### Task 4: Make Compilation And The Minimal Simulation Green

**Files:**

- Modify: scripts/create_tmpsim_online_model.m
- Modify: tests/TestOnlineModelSkeleton.m
- Regenerate: model/tmpsim_online.slx

- [ ] **Step 1: Correct only model-generation details exposed by the red tests**

Run the builder, use update-diagram diagnostics, and fix direct builder causes. Every bus Inport/Outport must use the concrete type declared in Task 2's port table: Bus: RefBus, Bus: MeasBus, Bus: ErrBus, Bus: HistoryBus, Bus: RiskRawBus, Bus: RiskBus, Bus: MPCParamBus, or Bus: CmdBus. Every input must have an internal terminator or top-level signal, all Bus Creator element inputs must match the Day 2 contract, and every relevant mock block/port must own the stated sample time. Do not change any Data Dictionary bus definition.

- [ ] **Step 2: Verify all Day 3 tests pass**

Run:

~~~powershell
& 'G:\Matlab2024b\bin\matlab.exe' -batch "cd('D:\Desktop\TransformerMPC'); results=runtests('tests/TestOnlineModelSkeleton.m'); assertSuccess(results)"
~~~

Expected: all Day 3 tests pass, update diagram reports no Bus type error, no sample-time conflict, and no unconnected-port error, then the 0.2-second simulation exits normally.

- [ ] **Step 3: Commit the green verification cycle**

~~~powershell
git add scripts/create_tmpsim_online_model.m tests/TestOnlineModelSkeleton.m model/tmpsim_online.slx
git commit -m "test: verify day 3 model smoke simulation"
~~~

### Task 5: Document, Verify Day 2 Regression, And Commit Locally

**Files:**

- Create: docs/Day_03_Handoff.md
- Modify: model/README.md
- Test: tests/TestSignalContracts.m
- Test: tests/TestOnlineModelSkeleton.m

- [ ] **Step 1: Update model/README.md**

Append:

~~~markdown
## Day 3 online skeleton

tmpsim_online.slx is generated by scripts/create_tmpsim_online_model.m and uses tmpsim_online.sldd. It is a fixed-step, mock-only top-level skeleton at Ts_mpc=0.02 s, with TransformerRisk mock output at Ts_tr=0.10 s.

Run from the repository root:

    addpath('scripts'); addpath('config'); addpath('src');
    create_tmpsim_online_model();

It intentionally contains no CarSim S-Function, CarSimAdapter.m, Transformer inference, Risk Supervisor algorithm, MPC, or speed-control implementation.
~~~

- [ ] **Step 2: Create docs/Day_03_Handoff.md**

After the final verification, document the model/builder/test paths, nine subsystem names, Day 2 Bus source, 0.02-second fixed model rate, 0.10-second Transformer mock rate, the later CarSim integration step of 0.0005 seconds and output log step of 0.02 seconds, exact passing commands/results, environment release, scope exclusions, and Day 4's explicit CarSimAdapter mapping responsibility. State that the branch is committed locally and not pushed.

- [ ] **Step 3: Run the complete Day 2 plus Day 3 verification**

Run:

~~~powershell
& 'G:\Matlab2024b\bin\matlab.exe' -batch "cd('D:\Desktop\TransformerMPC'); addpath('scripts'); addpath('config'); addpath('src'); create_tmpsim_dictionary(); cfg=tmpsim_config(); tmpsim.validateOnlineConfig(cfg); results=runtests({'tests/TestSignalContracts.m','tests/TestOnlineModelSkeleton.m'}); assertSuccess(results); create_tmpsim_online_model(); load_system('model/tmpsim_online.slx'); set_param('tmpsim_online','SimulationCommand','update'); out=sim('model/tmpsim_online.slx','StopTime','0.20'); assert(out.SimulationMetadata.ModelInfo.StopTime >= 0.20); close_system('tmpsim_online',0);"
~~~

Expected: MATLAB exits code 0; all tests pass; all model compilation diagnostics are clean; a 0.2-second mock simulation completes.

- [ ] **Step 4: Run repository hygiene checks**

~~~powershell
git diff --check
git status --short
git log --oneline --decorate -3
~~~

Expected: no whitespace errors and no generated cache, logs, CarSim installation files, or result files.

- [ ] **Step 5: Commit verified Day 3 deliverables without pushing**

~~~powershell
git add scripts/create_tmpsim_online_model.m tests/TestOnlineModelSkeleton.m model/tmpsim_online.slx model/README.md docs/Day_03_Handoff.md
git commit -m "docs: hand off day 3 model skeleton"
git status --short --branch
~~~

Expected: a clean local Day 3 branch. Do not run git push.

## Self-Review

Task 2 covers model creation, data-dictionary association, all nine named top-level subsystems, Day 2 buses, standard placeholder blocks, and fixed sample times. Task 3/4 implements test-first checks and Simulink compilation/minimal simulation verification. Task 5 records the required later CarSim time steps, handoff content, complete Day 2/3 regression, local commit, and no-push constraint.

No Day 4 CarSim adapter, physical integration, Transformer inference, Risk Supervisor, MPC, or speed-control algorithm appears in the plan. All Bus types are Day 2 bus names, all timing is assigned to blocks/ports rather than BusElement, and every wiring/table reference uses the same port/type names.
