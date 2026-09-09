function modelPath = create_tmpsim_online_model()
%CREATE_TMPSIM_ONLINE_MODEL Creates the Day 3 mock-only online skeleton.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
modelFolder = fullfile(projectRoot, 'model');
modelName = 'tmpsim_online';
modelPath = fullfile(modelFolder, [modelName '.slx']);

addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'scripts'));
addpath(fullfile(projectRoot, 'src'));

cfg = tmpsim_config();
tmpsim.validateOnlineConfig(cfg);

initialFolder = pwd;
restoreFolder = onCleanup(@() cd(initialFolder));
cd(modelFolder);

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if isfile(modelPath)
    delete(modelPath);
end

new_system(modelName);
set_param(modelName, ...
    'DataDictionary', 'tmpsim_online.sldd', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(cfg.sample.Ts_mpc), ...
    'StopTime', '0.20', ...
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
clear restoreFolder
end

function buildScenarioManager(modelName, cfg)
subsystem = createSubsystem(modelName, 'ScenarioManager', [30 30 180 150]);
[names, values, types] = busSpecification('RefBus', cfg);
addBusSource(subsystem, 'RefBus', names, values, types, cfg.sample.Ts_mpc, ...
    'ref_bus');
end

function buildCarSimAdapter(modelName, cfg)
subsystem = createSubsystem(modelName, 'CarSimAdapter', [1160 230 1310 350]);
addBusInput(subsystem, 'cmd_bus', 'CmdBus', [30 35 60 55]);
addTerminator(subsystem, 'cmd_bus_unused', [130 35 160 55]);
add_line(subsystem, 'cmd_bus/1', 'cmd_bus_unused/1', 'autorouting', 'on');

[names, values, types] = busSpecification('MeasBus', cfg);
addBusSource(subsystem, 'MeasBus', names, values, types, cfg.sample.Ts_mpc, ...
    'meas_bus');
addDelayPlaceholder(subsystem, cfg.sample.Ts_mpc, 470);
end

function buildErrorFeatureBuilder(modelName, cfg)
subsystem = createSubsystem(modelName, 'ErrorFeatureBuilder', [250 30 400 180]);
addBusInput(subsystem, 'ref_bus', 'RefBus', [30 30 60 50]);
addBusInput(subsystem, 'meas_bus', 'MeasBus', [30 75 60 95]);
terminateInput(subsystem, 'ref_bus', [130 30 160 50]);
terminateInput(subsystem, 'meas_bus', [130 75 160 95]);

[names, values, types] = busSpecification('ErrBus', cfg);
addBusSource(subsystem, 'ErrBus', names, values, types, cfg.sample.Ts_mpc, ...
    'err_bus');
addScalarSource(subsystem, 'latest_feature_value', 'zeros(1, 14)', 'double', ...
    cfg.sample.Ts_mpc, 'latest_feature', [35 460 135 480], [330 460 360 480]);
end

function buildHistoryBuffer(modelName, cfg)
subsystem = createSubsystem(modelName, 'HistoryBuffer', [470 30 620 150]);
addScalarInput(subsystem, 'latest_feature', 'double', '[1 14]', [30 35 60 55]);
addTerminator(subsystem, 'latest_feature_unused', [130 35 160 55]);
add_line(subsystem, 'latest_feature/1', 'latest_feature_unused/1', ...
    'autorouting', 'on');

[names, values, types] = busSpecification('HistoryBus', cfg);
addBusSource(subsystem, 'HistoryBus', names, values, types, cfg.sample.Ts_mpc, ...
    'history_bus');
addDelayPlaceholder(subsystem, cfg.sample.Ts_mpc, 380);
end

function buildTransformerRisk(modelName, cfg)
subsystem = createSubsystem(modelName, 'TransformerRisk', [690 30 840 150]);
addBusInput(subsystem, 'history_bus', 'HistoryBus', [30 35 60 55]);
addTerminator(subsystem, 'history_bus_unused', [130 35 160 55]);
add_line(subsystem, 'history_bus/1', 'history_bus_unused/1', ...
    'autorouting', 'on');

[names, values, types] = busSpecification('RiskRawBus', cfg);
addBusSource(subsystem, 'RiskRawBus', names, values, types, cfg.sample.Ts_tr, ...
    'risk_raw_bus');
addDelayPlaceholder(subsystem, cfg.sample.Ts_tr, 425);
end

function buildRiskSupervisor(modelName, cfg)
subsystem = createSubsystem(modelName, 'RiskSupervisor', [910 30 1060 210]);
addBusInput(subsystem, 'risk_raw_bus', 'RiskRawBus', [30 25 60 45]);
addBusInput(subsystem, 'err_bus', 'ErrBus', [30 65 60 85]);
addBusInput(subsystem, 'meas_bus', 'MeasBus', [30 105 60 125]);
addBusInput(subsystem, 'ref_bus', 'RefBus', [30 145 60 165]);
terminateInput(subsystem, 'risk_raw_bus', [130 25 160 45]);
terminateInput(subsystem, 'err_bus', [130 65 160 85]);
terminateInput(subsystem, 'meas_bus', [130 105 160 125]);
terminateInput(subsystem, 'ref_bus', [130 145 160 165]);

[names, values, types] = busSpecification('RiskBus', cfg);
addBusSource(subsystem, 'RiskBus', names, values, types, cfg.sample.Ts_mpc, ...
    'risk_bus');
[names, values, types] = busSpecification('MPCParamBus', cfg);
addBusSource(subsystem, 'MPCParamBus', names, values, types, ...
    cfg.sample.Ts_mpc, 'mpc_param_bus');
addScalarSource(subsystem, 'v_ref_adapt_value', '0.0', 'double', ...
    cfg.sample.Ts_mpc, 'v_ref_adapt', [35 605 135 625], [330 605 360 625]);
end

function buildLateralControllerVariant(modelName, cfg)
subsystem = createSubsystem(modelName, 'LateralControllerVariant', ...
    [1160 30 1310 190]);
addBusInput(subsystem, 'err_bus', 'ErrBus', [30 25 60 45]);
addBusInput(subsystem, 'meas_bus', 'MeasBus', [30 65 60 85]);
addBusInput(subsystem, 'ref_bus', 'RefBus', [30 105 60 125]);
addBusInput(subsystem, 'risk_bus', 'RiskBus', [30 145 60 165]);
addBusInput(subsystem, 'mpc_param_bus', 'MPCParamBus', [30 185 60 205]);
terminateInput(subsystem, 'err_bus', [130 25 160 45]);
terminateInput(subsystem, 'meas_bus', [130 65 160 85]);
terminateInput(subsystem, 'ref_bus', [130 105 160 125]);
terminateInput(subsystem, 'risk_bus', [130 145 160 165]);
terminateInput(subsystem, 'mpc_param_bus', [130 185 160 205]);

addScalarSource(subsystem, 'delta_cmd_value', '0.0', 'double', ...
    cfg.sample.Ts_mpc, 'delta_cmd', [35 260 135 280], [330 260 360 280]);
addScalarSource(subsystem, 'mpc_feasible_value', 'false', 'boolean', ...
    cfg.sample.Ts_mpc, 'mpc_feasible', [35 300 135 320], [330 300 360 320]);
addScalarSource(subsystem, 'mpc_status_code_value', 'uint8(0)', 'uint8', ...
    cfg.sample.Ts_mpc, 'mpc_status_code', [35 340 135 360], [330 340 360 360]);
addScalarSource(subsystem, 'solve_time_ms_value', '0.0', 'double', ...
    cfg.sample.Ts_mpc, 'solve_time_ms', [35 380 135 400], [330 380 360 400]);
end

function buildSpeedOuterLoop(modelName, cfg)
subsystem = createSubsystem(modelName, 'SpeedOuterLoop', [1160 430 1310 540]);
addBusInput(subsystem, 'meas_bus', 'MeasBus', [30 30 60 50]);
addScalarInput(subsystem, 'v_ref_adapt', 'double', '1', [30 75 60 95]);
terminateInput(subsystem, 'meas_bus', [130 30 160 50]);
terminateInput(subsystem, 'v_ref_adapt', [130 75 160 95]);
addScalarSource(subsystem, 'ax_cmd_value', '0.0', 'double', ...
    cfg.sample.Ts_mpc, 'ax_cmd', [35 160 135 180], [330 160 360 180]);
addScalarSource(subsystem, 'speed_status_code_value', 'uint8(0)', 'uint8', ...
    cfg.sample.Ts_mpc, 'speed_status_code', [35 200 135 220], [330 200 360 220]);
end

function buildLogger(modelName, ~)
subsystem = createSubsystem(modelName, 'Logger', [1380 140 1530 350]);
inputSpecs = { ...
    'ref_bus', 'RefBus'; ...
    'meas_bus', 'MeasBus'; ...
    'err_bus', 'ErrBus'; ...
    'history_bus', 'HistoryBus'; ...
    'risk_raw_bus', 'RiskRawBus'; ...
    'risk_bus', 'RiskBus'; ...
    'mpc_param_bus', 'MPCParamBus'; ...
    'cmd_bus', 'CmdBus'};
for index = 1:size(inputSpecs, 1)
    y = 25 + 40 * (index - 1);
    addBusInput(subsystem, inputSpecs{index, 1}, inputSpecs{index, 2}, ...
        [30 y 60 y + 20]);
    terminateInput(subsystem, inputSpecs{index, 1}, [130 y 160 y + 20]);
end
end

function buildCommandAssembler(modelName, cfg)
blockPath = [modelName '/CommandAssembler'];
add_block('simulink/Signal Routing/Bus Creator', blockPath, ...
    'Inputs', '5', 'OutDataTypeStr', 'Bus: CmdBus', 'NonVirtualBus', 'on', ...
    'Position', [940 310 970 360]);
addConstant(modelName, 'cmd_valid_value', 'true', 'boolean', ...
    cfg.sample.Ts_mpc, [800 365 900 385]);
addConstant(modelName, 'hold_last_cmd_value', 'false', 'boolean', ...
    cfg.sample.Ts_mpc, [800 405 900 425]);
addConstant(modelName, 'control_mode_id_value', 'uint8(1)', 'uint8', ...
    cfg.sample.Ts_mpc, [800 445 900 465]);
setNamedLine(modelName, 'cmd_valid_value/1', 'CommandAssembler/3', ...
    'cmd_valid');
setNamedLine(modelName, 'hold_last_cmd_value/1', 'CommandAssembler/4', ...
    'hold_last_cmd');
setNamedLine(modelName, 'control_mode_id_value/1', 'CommandAssembler/5', ...
    'control_mode_id');
end

function wireTopLevel(modelName)
connections = { ...
    'ScenarioManager/1', 'ErrorFeatureBuilder/1'; ...
    'ScenarioManager/1', 'RiskSupervisor/4'; ...
    'ScenarioManager/1', 'Logger/1'; ...
    'CarSimAdapter/1', 'ErrorFeatureBuilder/2'; ...
    'CarSimAdapter/1', 'RiskSupervisor/3'; ...
    'CarSimAdapter/1', 'LateralControllerVariant/2'; ...
    'CarSimAdapter/1', 'SpeedOuterLoop/1'; ...
    'CarSimAdapter/1', 'Logger/2'; ...
    'ErrorFeatureBuilder/1', 'RiskSupervisor/2'; ...
    'ErrorFeatureBuilder/1', 'LateralControllerVariant/1'; ...
    'ErrorFeatureBuilder/1', 'Logger/3'; ...
    'ErrorFeatureBuilder/2', 'HistoryBuffer/1'; ...
    'HistoryBuffer/1', 'TransformerRisk/1'; ...
    'HistoryBuffer/1', 'Logger/4'; ...
    'TransformerRisk/1', 'RiskSupervisor/1'; ...
    'TransformerRisk/1', 'Logger/5'; ...
    'RiskSupervisor/1', 'LateralControllerVariant/4'; ...
    'RiskSupervisor/1', 'Logger/6'; ...
    'RiskSupervisor/2', 'LateralControllerVariant/5'; ...
    'RiskSupervisor/2', 'Logger/7'; ...
    'RiskSupervisor/3', 'SpeedOuterLoop/2'; ...
    'CommandAssembler/1', 'CarSimAdapter/1'; ...
    'CommandAssembler/1', 'Logger/8'};
for index = 1:size(connections, 1)
    add_line(modelName, connections{index, 1}, connections{index, 2}, ...
        'autorouting', 'on');
end
setNamedLine(modelName, 'LateralControllerVariant/1', 'CommandAssembler/1', ...
    'delta_cmd');
setNamedLine(modelName, 'SpeedOuterLoop/1', 'CommandAssembler/2', 'ax_cmd');

addTopLevelTerminator(modelName, 'mpc_feasible_sink', [1360 35 1390 55]);
addTopLevelTerminator(modelName, 'mpc_status_sink', [1360 75 1390 95]);
addTopLevelTerminator(modelName, 'solve_time_sink', [1360 115 1390 135]);
addTopLevelTerminator(modelName, 'speed_status_sink', [1360 400 1390 420]);
add_line(modelName, 'LateralControllerVariant/2', 'mpc_feasible_sink/1', ...
    'autorouting', 'on');
add_line(modelName, 'LateralControllerVariant/3', 'mpc_status_sink/1', ...
    'autorouting', 'on');
add_line(modelName, 'LateralControllerVariant/4', 'solve_time_sink/1', ...
    'autorouting', 'on');
add_line(modelName, 'SpeedOuterLoop/2', 'speed_status_sink/1', ...
    'autorouting', 'on');
end

function subsystem = createSubsystem(modelName, name, position)
subsystem = [modelName '/' name];
add_block('simulink/Ports & Subsystems/Subsystem', subsystem, ...
    'Position', position);
delete_block([subsystem '/In1']);
delete_block([subsystem '/Out1']);
end

function addBusSource(subsystem, busName, elementNames, values, types, ...
        sampleTime, outputName)
creatorName = [busName ' Mock'];
add_block('simulink/Signal Routing/Bus Creator', [subsystem '/' creatorName], ...
    'Inputs', num2str(numel(elementNames)), ...
    'OutDataTypeStr', ['Bus: ' busName], 'NonVirtualBus', 'on', ...
    'Position', [230 35 260 55]);
for index = 1:numel(elementNames)
    y = 25 + 45 * (index - 1);
    constantName = [busName '_' elementNames{index}];
    addConstant(subsystem, constantName, values{index}, types{index}, ...
        sampleTime, [35 y 135 y + 20]);
    lineHandle = add_line(subsystem, [constantName '/1'], ...
        [creatorName '/' num2str(index)], 'autorouting', 'on');
    set_param(lineHandle, 'Name', elementNames{index});
end
addBusOutput(subsystem, outputName, busName, sampleTime, [330 35 360 55]);
add_line(subsystem, [creatorName '/1'], [outputName '/1'], ...
    'autorouting', 'on');
end

function addBusInput(subsystem, name, busName, position)
add_block('simulink/Ports & Subsystems/In1', [subsystem '/' name], ...
    'OutDataTypeStr', ['Bus: ' busName], 'SampleTime', '-1', ...
    'Position', position);
end

function addScalarInput(subsystem, name, dataType, dimensions, position)
add_block('simulink/Ports & Subsystems/In1', [subsystem '/' name], ...
    'OutDataTypeStr', dataType, 'PortDimensions', dimensions, ...
    'SampleTime', '-1', 'Position', position);
end

function addBusOutput(subsystem, name, busName, sampleTime, position)
add_block('simulink/Ports & Subsystems/Out1', [subsystem '/' name], ...
    'OutDataTypeStr', ['Bus: ' busName], ...
    'SampleTime', num2str(sampleTime), 'Position', position);
end

function addScalarSource(subsystem, constantName, value, dataType, ...
        sampleTime, outputName, constantPosition, outputPosition)
addConstant(subsystem, constantName, value, dataType, sampleTime, ...
    constantPosition);
add_block('simulink/Ports & Subsystems/Out1', [subsystem '/' outputName], ...
    'OutDataTypeStr', dataType, 'SampleTime', num2str(sampleTime), ...
    'Position', outputPosition);
add_line(subsystem, [constantName '/1'], [outputName '/1'], ...
    'autorouting', 'on');
end

function addConstant(parent, name, value, dataType, sampleTime, position)
add_block('simulink/Sources/Constant', [parent '/' name], ...
    'Value', value, 'OutDataTypeStr', dataType, ...
    'SampleTime', num2str(sampleTime), 'Position', position);
end

function terminateInput(subsystem, inputName, position)
terminatorName = [inputName '_unused'];
addTerminator(subsystem, terminatorName, position);
add_line(subsystem, [inputName '/1'], [terminatorName '/1'], ...
    'autorouting', 'on');
end

function addTerminator(parent, name, position)
add_block('simulink/Sinks/Terminator', [parent '/' name], ...
    'Position', position);
end

function addTopLevelTerminator(modelName, name, position)
addTerminator(modelName, name, position);
end

function setNamedLine(modelName, source, destination, signalName)
lineHandle = add_line(modelName, source, destination, 'autorouting', 'on');
set_param(lineHandle, 'Name', signalName);
end

function addDelayPlaceholder(subsystem, sampleTime, y)
addConstant(subsystem, 'delay_seed', '0.0', 'double', sampleTime, ...
    [35 y 135 y + 20]);
add_block('simulink/Discrete/Unit Delay', [subsystem '/state_hold'], ...
    'SampleTime', num2str(sampleTime), 'Position', [190 y 220 y + 20]);
addTerminator(subsystem, 'state_hold_unused', [275 y 305 y + 20]);
add_line(subsystem, 'delay_seed/1', 'state_hold/1', 'autorouting', 'on');
add_line(subsystem, 'state_hold/1', 'state_hold_unused/1', ...
    'autorouting', 'on');
end

function [names, values, types] = busSpecification(busName, cfg)
switch busName
    case 'RefBus'
        names = {'x_ref', 'y_ref', 'psi_ref', 'kappa_ref', 'v_ref_base', ...
            'path_idx', 'ref_valid'};
        values = {'0.0', '0.0', '0.0', '0.0', '0.0', 'uint32(0)', 'false'};
        types = {'double', 'double', 'double', 'double', 'double', ...
            'uint32', 'boolean'};
    case 'MeasBus'
        names = {'x', 'y', 'yaw', 'vx', 'vy', 'yaw_rate', 'beta', 'ay', ...
            'delta_meas', 'ax_meas', 'delta_rate_meas', 'meas_valid'};
        values = [repmat({'0.0'}, 1, 11), {'false'}];
        types = [repmat({'double'}, 1, 11), {'boolean'}];
    case 'ErrBus'
        names = {'e_y', 'e_psi', 'e_y_rate', 'e_psi_rate', 'yaw_ref', ...
            'kappa_ref', 'v_ref_base', 'err_valid'};
        values = [repmat({'0.0'}, 1, 7), {'false'}];
        types = [repmat({'double'}, 1, 7), {'boolean'}];
    case 'HistoryBus'
        names = {'window', 'latest_feature', 'window_ready', 'history_len', ...
            'sample_time'};
        values = {'zeros(16, 14)', 'zeros(1, 14)', 'false', 'uint16(0)', ...
            num2str(cfg.sample.Ts_mpc)};
        types = {'double', 'double', 'boolean', 'uint16', 'double'};
    case 'RiskRawBus'
        names = {'r_low_raw', 'r_ey_raw', 'r_stab_raw', 'k_v_raw', ...
            'risk_raw_valid', 'risk_source_id', 'transformer_status_code', ...
            'transformer_latency_ms'};
        values = {'0.0', '0.0', '0.0', '1.0', 'false', 'uint8(0)', ...
            'uint8(1)', '0.0'};
        types = {'double', 'double', 'double', 'double', 'boolean', ...
            'uint8', 'uint8', 'double'};
    case 'RiskBus'
        names = {'r_low', 'r_ey', 'r_stab', 'k_v', 'risk_valid', ...
            'degraded_mode', 'risk_source_id'};
        values = {'0.0', '0.0', '0.0', '1.0', 'true', 'false', 'uint8(0)'};
        types = {'double', 'double', 'double', 'double', 'boolean', ...
            'boolean', 'uint8'};
    case 'MPCParamBus'
        names = {'q_y', 'q_psi', 'q_beta', 'q_r', 'r_delta', 'r_d_delta', ...
            'beta_max', 'yaw_rate_max', 'ay_max', 'delta_max', ...
            'delta_rate_max', 'v_ref_adapt', 'param_valid'};
        values = {num2str(cfg.mpc.q_y0), num2str(cfg.mpc.q_psi0), ...
            num2str(cfg.mpc.q_beta0), num2str(cfg.mpc.q_r0), ...
            num2str(cfg.mpc.r_delta0), num2str(cfg.mpc.r_d_delta0), ...
            num2str(cfg.mpc.beta_max0), num2str(cfg.mpc.yaw_rate_max0), ...
            num2str(cfg.mpc.ay_max0), num2str(cfg.mpc.delta_max), ...
            num2str(cfg.mpc.delta_rate_max), '0.0', 'true'};
        types = [repmat({'double'}, 1, 12), {'boolean'}];
    otherwise
        error('tmpsim:UnknownBusSpecification', ...
            'No mock specification is defined for %s.', busName);
end
end
