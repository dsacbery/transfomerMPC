function summary = verifyCarSimStraightSmoke(log, sampleTime, maxYawStepRad)
%VERIFYCARSIMSTRAIGHTSMOKE Checks the Day 4 straight-line acceptance signals.

validateattributes(sampleTime, {'double'}, {'scalar', 'finite', 'positive'});
validateattributes(maxYawStepRad, {'double'}, ...
    {'scalar', 'finite', 'nonnegative'});

requiredFields = {'time', 'x', 'vx', 'yaw', 'delta_cmd', ...
    'throttle_cmd', 'brake_cmd'};
if ~isstruct(log) || ~isscalar(log) || ~all(isfield(log, requiredFields))
    error('tmpsim:InvalidStraightSmokeLog', ...
        'The straight-smoke log is missing one or more required fields.');
end

values = cellfun(@(name) log.(name), requiredFields, 'UniformOutput', false);
sampleCount = numel(log.time);
if sampleCount < 2 || any(cellfun(@(value) ~isnumeric(value) ...
        || ~isreal(value) || ~isvector(value) || numel(value) ~= sampleCount ...
        || any(~isfinite(value)), values))
    error('tmpsim:InvalidStraightSmokeLog', ...
        'Straight-smoke fields must be finite vectors with a shared length.');
end

time = log.time(:);
x = log.x(:);
vx = log.vx(:);
yaw = log.yaw(:);
deltaCmd = log.delta_cmd(:);
throttleCmd = log.throttle_cmd(:);
brakeCmd = log.brake_cmd(:);

timeTolerance = max(1e-12, eps(sampleTime) * 16);
if any(abs(diff(time) - sampleTime) > timeTolerance) || time(end) - time(1) < 10
    error('tmpsim:StraightSmokeTimingFailure', ...
        'The straight-smoke log must cover at least 10 s at the expected rate.');
end
if any(diff(x) < 0) || ~any(diff(x) > 0)
    error('tmpsim:StraightSmokePositionFailure', ...
        'x must increase monotonically during the straight smoke test.');
end
if any(vx <= 0)
    error('tmpsim:StraightSmokeVelocityFailure', ...
        'vx must remain positive during the straight smoke test.');
end
if any(abs(diff(yaw)) > maxYawStepRad)
    error('tmpsim:StraightSmokeYawFailure', ...
        'yaw contains a step larger than the configured smoke-test threshold.');
end
if any(deltaCmd ~= 0)
    error('tmpsim:StraightSmokeSteeringFailure', ...
        'delta_cmd must remain zero for the straight smoke test.');
end
if any(throttleCmd > 0 & brakeCmd > 0)
    error('tmpsim:StraightSmokeActuatorFailure', ...
        'throttle_cmd and brake_cmd cannot both be positive.');
end
summary = struct( ...
    'duration_s', time(end) - time(1), ...
    'sample_count', uint32(sampleCount));
end
