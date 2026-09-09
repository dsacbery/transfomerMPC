classdef CarSimAdapter < handle
    %CARSIMADAPTER Converts named CarSim signals at the P0 plant boundary.

    properties (SetAccess = private)
        TsMpc
        CommandMap
        PreviousDelta = 0
        HasPreviousDelta = false
        LastPlantCommand
    end

    methods
        function obj = CarSimAdapter(tsMpc, commandMap)
            validateattributes(tsMpc, {'double'}, {'scalar', 'finite', ...
                'positive'});
            if abs(tsMpc - 0.02) > 1e-12
                error('tmpsim:InvalidAdapterSampleTime', ...
                    'CarSimAdapter requires the frozen Ts_mpc value of 0.02 s.');
            end

            obj.TsMpc = tsMpc;
            obj.CommandMap = commandMap;
            obj.LastPlantCommand = zeroPlantCommand();
        end

        function reset(obj)
            obj.PreviousDelta = 0;
            obj.HasPreviousDelta = false;
            obj.LastPlantCommand = zeroPlantCommand();
        end

        function meas = adaptMeasurement(obj, raw)
            [values, valid] = readMeasurementValues(raw);
            if ~valid
                obj.PreviousDelta = 0;
                obj.HasPreviousDelta = false;
                meas = zeroMeasurement();
                return
            end

            meas = zeroMeasurement();
            meas.x = values.Xo;
            meas.y = values.Yo;
            meas.yaw = deg2rad(values.Yaw);
            meas.vx = values.Vx / 3.6;
            meas.vy = values.Vy / 3.6;
            meas.yaw_rate = deg2rad(values.AVz);
            meas.beta = deg2rad(values.Beta);
            meas.ay = values.Ay * 9.80665;
            meas.delta_meas = deg2rad((values.Steer_L1 + values.Steer_R1) / 2);
            meas.ax_meas = values.Ax * 9.80665;
            if obj.HasPreviousDelta
                meas.delta_rate_meas = (meas.delta_meas - obj.PreviousDelta) ...
                    / obj.TsMpc;
            end
            meas.meas_valid = valid;

            obj.PreviousDelta = meas.delta_meas;
            obj.HasPreviousDelta = true;
        end

        function plant = adaptCommand(obj, cmd)
            [values, valid] = readCommandValues(cmd);
            if ~valid || ~isValidCommandMap(obj.CommandMap) ...
                    || ~values.cmd_valid || values.hold_last_cmd
                plant = obj.LastPlantCommand;
                return
            end

            deltaCmd = clip(values.delta_cmd, obj.CommandMap.delta_min_rad, ...
                obj.CommandMap.delta_max_rad);
            axCmd = clip(values.ax_cmd, obj.CommandMap.ax_min_mps2, ...
                obj.CommandMap.ax_max_mps2);

            plant = zeroPlantCommand();
            plant.IMP_STEER_L1 = rad2deg(deltaCmd);
            plant.IMP_STEER_R1 = rad2deg(deltaCmd);
            if axCmd >= 0
                plant.IMP_THROTTLE_ENGINE = axCmd ...
                    / obj.CommandMap.ax_max_mps2 ...
                    * obj.CommandMap.throttle_max;
            else
                plant.IMP_PCON_BK = -axCmd ...
                    / -obj.CommandMap.ax_min_mps2 ...
                    * obj.CommandMap.brake_max_mpa;
            end
            obj.LastPlantCommand = plant;
        end
    end
end

function [values, valid] = readMeasurementValues(raw)
values = struct('Xo', 0, 'Yo', 0, 'Yaw', 0, 'Vx', 0, 'Vy', 0, ...
    'AVz', 0, 'Beta', 0, 'Ay', 0, 'Steer_L1', 0, 'Steer_R1', 0, 'Ax', 0);
valid = isstruct(raw) && isscalar(raw);
if ~valid
    return
end

fieldNames = {'Xo', 'Yo', 'Yaw', 'Vx', 'Vy', 'AVz', 'Beta', ...
    'Steer_L1', 'Steer_R1', 'Ax'};
for index = 1:numel(fieldNames)
    fieldName = fieldNames{index};
    [values.(fieldName), fieldValid] = readFiniteScalar(raw, fieldName);
    valid = valid && fieldValid;
end

[values.Ay, ayValid] = readFiniteScalar(raw, 'Ay');
valid = valid && ayValid;
end

function [value, valid] = readFiniteScalar(raw, fieldName)
value = 0;
valid = isfield(raw, fieldName);
if ~valid
    return
end

candidate = raw.(fieldName);
valid = isnumeric(candidate) && isreal(candidate) && isscalar(candidate) ...
    && isfinite(candidate);
if valid
    value = double(candidate);
end
end

function meas = zeroMeasurement()
meas = struct( ...
    'x', 0.0, ...
    'y', 0.0, ...
    'yaw', 0.0, ...
    'vx', 0.0, ...
    'vy', 0.0, ...
    'yaw_rate', 0.0, ...
    'beta', 0.0, ...
    'ay', 0.0, ...
    'delta_meas', 0.0, ...
    'ax_meas', 0.0, ...
    'delta_rate_meas', 0.0, ...
    'meas_valid', false);
end

function plant = zeroPlantCommand()
plant = struct( ...
    'IMP_STEER_L1', 0.0, ...
    'IMP_STEER_R1', 0.0, ...
    'IMP_THROTTLE_ENGINE', 0.0, ...
    'IMP_PCON_BK', 0.0);
end

function [values, valid] = readCommandValues(cmd)
values = struct('delta_cmd', 0.0, 'ax_cmd', 0.0, 'cmd_valid', false, ...
    'hold_last_cmd', false);
valid = isstruct(cmd) && isscalar(cmd);
if ~valid
    return
end

[values.delta_cmd, deltaValid] = readFiniteScalar(cmd, 'delta_cmd');
[values.ax_cmd, axValid] = readFiniteScalar(cmd, 'ax_cmd');
[values.cmd_valid, cmdValid] = readLogicalScalar(cmd, 'cmd_valid');
[values.hold_last_cmd, holdValid] = readLogicalScalar(cmd, 'hold_last_cmd');
valid = deltaValid && axValid && cmdValid && holdValid;
end

function [value, valid] = readLogicalScalar(inputValue, fieldName)
value = false;
valid = isfield(inputValue, fieldName);
if ~valid
    return
end

candidate = inputValue.(fieldName);
valid = islogical(candidate) && isscalar(candidate);
if valid
    value = candidate;
end
end

function valid = isValidCommandMap(commandMap)
requiredFields = {'delta_min_rad', 'delta_max_rad', 'ax_min_mps2', ...
    'ax_max_mps2', 'throttle_max', 'brake_max_mpa'};
valid = isstruct(commandMap) && isscalar(commandMap) ...
    && all(isfield(commandMap, requiredFields));
if ~valid
    return
end

values = cellfun(@(name) commandMap.(name), requiredFields, ...
    'UniformOutput', false);
valid = all(cellfun(@(value) isnumeric(value) && isreal(value) ...
    && isscalar(value) && isfinite(value), values));
if ~valid
    return
end

valid = commandMap.delta_min_rad <= commandMap.delta_max_rad ...
    && commandMap.ax_min_mps2 < 0 ...
    && commandMap.ax_max_mps2 > 0 ...
    && commandMap.throttle_max >= 0 ...
    && commandMap.brake_max_mpa >= 0;
end

function value = clip(value, lowerBound, upperBound)
value = min(max(value, lowerBound), upperBound);
end
