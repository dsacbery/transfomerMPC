classdef ErrorFeatureBuilder < handle
    %ERRORFEATUREBUILDER Builds frozen Frenet errors and one feature row.

    properties (SetAccess = private)
        TsMpc
        PreviousEY = 0.0
    end

    methods
        function obj = ErrorFeatureBuilder(tsMpc)
            if ~(isa(tsMpc, 'double') && isreal(tsMpc) && isscalar(tsMpc) ...
                    && isfinite(tsMpc) && abs(tsMpc - 0.02) <= 1e-12)
                error('tmpsim:InvalidErrorFeatureSampleTime', ...
                    ['ErrorFeatureBuilder requires the frozen Ts_mpc value ' ...
                    'of 0.02 s.']);
            end

            obj.TsMpc = tsMpc;
        end

        function [err, latestFeature] = build(obj, ref, meas, resetRequested)
            if ~(islogical(resetRequested) && isscalar(resetRequested))
                error('tmpsim:InvalidErrorFeatureReset', ...
                    'resetRequested must be a logical scalar.');
            end
            if resetRequested
                obj.reset();
            end

            if ~isValidReference(ref) || ~isValidMeasurement(meas)
                err = zeroError();
                latestFeature = zeros(1, 14);
                return
            end

            dx = double(meas.x) - double(ref.x_ref);
            dy = double(meas.y) - double(ref.y_ref);
            eY = -sin(double(ref.psi_ref)) * dx ...
                + cos(double(ref.psi_ref)) * dy;
            ePsi = wrapAngle(double(meas.yaw) - double(ref.psi_ref));

            if double(meas.vx) < 0.5
                eYRate = clip((eY - obj.PreviousEY) / obj.TsMpc, -5.0, 5.0);
                errValid = false;
            else
                eYRate = double(meas.vy) + double(meas.vx) * sin(ePsi);
                errValid = logical(ref.ref_valid);
            end
            ePsiRate = double(meas.yaw_rate) ...
                - double(meas.vx) * double(ref.kappa_ref);

            err = struct( ...
                'e_y', eY, ...
                'e_psi', ePsi, ...
                'e_y_rate', eYRate, ...
                'e_psi_rate', ePsiRate, ...
                'yaw_ref', double(ref.psi_ref), ...
                'kappa_ref', double(ref.kappa_ref), ...
                'v_ref_base', double(ref.v_ref_base), ...
                'err_valid', logical(errValid));
            latestFeature = double([ ...
                meas.vx, meas.vy, meas.yaw_rate, meas.ay, meas.beta, ...
                meas.delta_meas, meas.delta_rate_meas, meas.ax_meas, ...
                eY, ePsi, eYRate, ePsiRate, ref.kappa_ref, ref.v_ref_base]);

            if ~all(isfinite(latestFeature))
                err = zeroError();
                latestFeature = zeros(1, 14);
                return
            end

            if logical(ref.ref_valid)
                obj.PreviousEY = eY;
            end
        end

        function reset(obj)
            obj.PreviousEY = 0.0;
        end
    end
end

function valid = isValidReference(ref)
fieldNames = {'x_ref', 'y_ref', 'psi_ref', 'kappa_ref', 'v_ref_base'};
valid = isstruct(ref) && isscalar(ref) && all(isfield(ref, fieldNames)) ...
    && isfield(ref, 'ref_valid');
if ~valid
    return
end

for index = 1:numel(fieldNames)
    valid = valid && isFiniteScalar(ref.(fieldNames{index}));
end
valid = valid && islogical(ref.ref_valid) && isscalar(ref.ref_valid);
end

function valid = isValidMeasurement(meas)
fieldNames = {'x', 'y', 'yaw', 'vx', 'vy', 'yaw_rate', 'beta', 'ay', ...
    'delta_meas', 'delta_rate_meas', 'ax_meas'};
valid = isstruct(meas) && isscalar(meas) && all(isfield(meas, fieldNames)) ...
    && isfield(meas, 'meas_valid');
if ~valid
    return
end

for index = 1:numel(fieldNames)
    valid = valid && isFiniteScalar(meas.(fieldNames{index}));
end
valid = valid && islogical(meas.meas_valid) && isscalar(meas.meas_valid) ...
    && meas.meas_valid;
end

function valid = isFiniteScalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) ...
    && isfinite(value);
end

function err = zeroError()
err = struct( ...
    'e_y', 0.0, ...
    'e_psi', 0.0, ...
    'e_y_rate', 0.0, ...
    'e_psi_rate', 0.0, ...
    'yaw_ref', 0.0, ...
    'kappa_ref', 0.0, ...
    'v_ref_base', 0.0, ...
    'err_valid', false);
end

function angle = wrapAngle(angle)
angle = atan2(sin(angle), cos(angle));
end

function value = clip(value, lowerBound, upperBound)
value = min(max(value, lowerBound), upperBound);
end
