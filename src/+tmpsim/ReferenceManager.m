classdef ReferenceManager < handle
    %REFERENCEMANAGER Selects forward-only references from a supplied path.

    properties (SetAccess = private)
        Path
        SpeedLimitMps
        VMinMps
        PathIndex = uint32(1)
    end

    methods
        function obj = ReferenceManager(pathXYpsiKappaV, speedLimitMps, vMinMps)
            if ~isValidPath(pathXYpsiKappaV)
                error('tmpsim:InvalidReferencePath', ...
                    'Reference path must be a finite N-by-5 double matrix.');
            end
            if ~isValidSpeedBounds(speedLimitMps, vMinMps)
                error('tmpsim:InvalidReferenceSpeedBounds', ...
                    ['Reference speed bounds must be finite scalars with ' ...
                    '0 <= vMinMps <= speedLimitMps and speedLimitMps > 0.']);
            end

            obj.Path = pathXYpsiKappaV;
            obj.SpeedLimitMps = double(speedLimitMps);
            obj.VMinMps = double(vMinMps);
        end

        function ref = step(obj, meas, resetRequested)
            if ~(islogical(resetRequested) && isscalar(resetRequested))
                error('tmpsim:InvalidReferenceReset', ...
                    'resetRequested must be a logical scalar.');
            end

            if resetRequested
                obj.reset();
            elseif isValidMeasurement(meas)
                obj.advanceToNearestForwardPoint(meas.x, meas.y);
            end

            ref = obj.currentReference();
        end

        function reset(obj)
            obj.PathIndex = uint32(1);
        end
    end

    methods (Access = private)
        function advanceToNearestForwardPoint(obj, x, y)
            startIndex = double(obj.PathIndex);
            endIndex = min(startIndex + 49, size(obj.Path, 1));
            candidates = obj.Path(startIndex:endIndex, 1:2);
            distances = hypot(candidates(:, 1) - double(x), ...
                candidates(:, 2) - double(y));
            [~, offset] = min(distances);
            obj.PathIndex = uint32(startIndex + offset - 1);
        end

        function ref = currentReference(obj)
            index = double(obj.PathIndex);
            point = obj.Path(index, :);
            baseSpeed = min(max(point(5), obj.VMinMps), obj.SpeedLimitMps);

            ref = struct( ...
                'x_ref', point(1), ...
                'y_ref', point(2), ...
                'psi_ref', point(3), ...
                'kappa_ref', point(4), ...
                'v_ref_base', baseSpeed, ...
                'path_idx', uint32(index), ...
                'ref_valid', logical(index < size(obj.Path, 1)));
        end
    end
end

function valid = isValidPath(pathXYpsiKappaV)
valid = isa(pathXYpsiKappaV, 'double') && isreal(pathXYpsiKappaV) ...
    && ismatrix(pathXYpsiKappaV) && ~isempty(pathXYpsiKappaV) ...
    && size(pathXYpsiKappaV, 2) == 5 ...
    && all(isfinite(pathXYpsiKappaV(:)));
end

function valid = isValidSpeedBounds(speedLimitMps, vMinMps)
valid = isFiniteScalar(speedLimitMps) && isFiniteScalar(vMinMps) ...
    && speedLimitMps > 0.0 && vMinMps >= 0.0 ...
    && vMinMps <= speedLimitMps;
end

function valid = isValidMeasurement(meas)
valid = isstruct(meas) && isscalar(meas) ...
    && isfield(meas, 'x') && isfield(meas, 'y') && isfield(meas, 'meas_valid') ...
    && isFiniteScalar(meas.x) && isFiniteScalar(meas.y);
if valid
    valid = islogical(meas.meas_valid) && isscalar(meas.meas_valid) ...
        && meas.meas_valid;
end
end

function valid = isFiniteScalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) ...
    && isfinite(value);
end
