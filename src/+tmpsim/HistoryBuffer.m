classdef HistoryBuffer < handle
    %HISTORYBUFFER Stores frozen feature rows in chronological window order.

    properties (SetAccess = private)
        TsMpc
        Buffer
        WriteIndex = uint16(1)
        HistoryLength = uint16(0)
    end

    methods
        function obj = HistoryBuffer(tsMpc)
            if ~(isa(tsMpc, 'double') && isreal(tsMpc) && isscalar(tsMpc) ...
                    && isfinite(tsMpc) && abs(tsMpc - 0.02) <= 1e-12)
                error('tmpsim:InvalidHistorySampleTime', ...
                    ['HistoryBuffer requires the frozen Ts_mpc value of ' ...
                    '0.02 s.']);
            end

            obj.TsMpc = tsMpc;
            obj.Buffer = zeros(16, 14);
        end

        function history = step(obj, latestFeature, resetRequested)
            if ~isValidFeature(latestFeature)
                error('tmpsim:InvalidHistoryFeature', ...
                    'latestFeature must be a finite 1-by-14 double row.');
            end
            if ~(islogical(resetRequested) && isscalar(resetRequested))
                error('tmpsim:InvalidHistoryReset', ...
                    'resetRequested must be a logical scalar.');
            end

            if resetRequested
                obj.reset();
            else
                obj.write(latestFeature);
            end

            history = obj.historyBus(latestFeature);
        end

        function reset(obj)
            obj.Buffer = zeros(16, 14);
            obj.WriteIndex = uint16(1);
            obj.HistoryLength = uint16(0);
        end
    end

    methods (Access = private)
        function write(obj, latestFeature)
            obj.Buffer(double(obj.WriteIndex), :) = latestFeature;
            obj.WriteIndex = uint16(mod(double(obj.WriteIndex), 16) + 1);
            if obj.HistoryLength < uint16(16)
                obj.HistoryLength = obj.HistoryLength + uint16(1);
            end
        end

        function history = historyBus(obj, latestFeature)
            if obj.HistoryLength < uint16(16)
                window = obj.Buffer;
            else
                oldestIndex = double(obj.WriteIndex);
                window = [obj.Buffer(oldestIndex:end, :); ...
                    obj.Buffer(1:oldestIndex - 1, :)];
            end

            history = struct( ...
                'window', window, ...
                'latest_feature', latestFeature, ...
                'window_ready', logical(obj.HistoryLength == uint16(16)), ...
                'history_len', obj.HistoryLength, ...
                'sample_time', obj.TsMpc);
        end
    end
end

function valid = isValidFeature(latestFeature)
valid = isa(latestFeature, 'double') && isreal(latestFeature) ...
    && isequal(size(latestFeature), [1 14]) ...
    && all(isfinite(latestFeature));
end
