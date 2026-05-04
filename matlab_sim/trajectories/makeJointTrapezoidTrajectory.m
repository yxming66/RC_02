function traj = makeJointTrapezoidTrajectory(q0, q1, maxVelocity, maxAcceleration, dt)
%MAKEJOINTTRAPEZOIDTRAJECTORY Synchronized trapezoid joint trajectory.
%
% This mirrors component/trajectory/trapezoid.hpp and
% component/trajectory/synchronized_profile.hpp.

if nargin < 5 || isempty(dt)
    dt = 0.02;
end

q0 = double(q0(:).');
q1 = double(q1(:).');
dof = numel(q0);
if numel(q1) ~= dof
    error("makeJointTrapezoidTrajectory:InvalidSize", ...
        "q0 and q1 must have the same size.");
end

maxVelocity = expandLimit(maxVelocity, dof, "maxVelocity");
maxAcceleration = expandLimit(maxAcceleration, dof, "maxAcceleration");
delta = q1 - q0;

minDurations = zeros(1, dof);
for i = 1:dof
    minDurations(i) = trapezoidMinDuration(delta(i), maxVelocity(i), maxAcceleration(i));
end
duration = max(minDurations);

if duration <= 1e-6
    time = 0.0;
else
    time = 0:dt:duration;
    if time(end) < duration
        time(end + 1) = duration; %#ok<AGROW>
    end
end

n = numel(time);
q = zeros(n, dof);
qd = zeros(n, dof);
qdd = zeros(n, dof);
profile = repmat(emptyAxisProfile(), 1, dof);

for i = 1:dof
    profile(i) = configureTrapezoidAxis(delta(i), maxVelocity(i), maxAcceleration(i), duration);
    for k = 1:n
        sample = sampleTrapezoidAxis(profile(i), time(k));
        q(k, i) = q0(i) + sample.position;
        qd(k, i) = sample.velocity;
        qdd(k, i) = sample.acceleration;
    end
end

q(end, :) = q1;
qd(end, :) = 0.0;
qdd(end, :) = 0.0;

traj = struct();
traj.type = "synchronized_trapezoid";
traj.time = time(:);
traj.q = q;
traj.qd = qd;
traj.qdd = qdd;
traj.duration = duration;
traj.minDurations = minDurations;
traj.profile = profile;
traj.maxVelocity = maxVelocity;
traj.maxAcceleration = maxAcceleration;
traj.valid = true;
end

function values = expandLimit(values, dof, name)
values = double(values(:).');
if isscalar(values)
    values = repmat(values, 1, dof);
end
if numel(values) ~= dof
    error("makeJointTrapezoidTrajectory:InvalidLimitSize", ...
        "%s must be scalar or have one value per joint.", name);
end
if any(values <= 0 | ~isfinite(values))
    error("makeJointTrapezoidTrajectory:InvalidLimit", ...
        "%s must be finite and positive.", name);
end
end

function duration = trapezoidMinDuration(distance, maxVelocity, maxAcceleration)
d = abs(distance);
v = abs(maxVelocity);
a = abs(maxAcceleration);
if d <= 1e-6
    duration = 0.0;
    return;
end
if v <= 1e-6 || a <= 1e-6
    duration = -1.0;
    return;
end

tAccel = v / a;
accelDecelDistance = v * v / a;
if d <= accelDecelDistance
    duration = 2.0 * sqrt(d / a);
else
    duration = 2.0 * tAccel + (d - accelDecelDistance) / v;
end
end

function profile = emptyAxisProfile()
profile = struct( ...
    "distance", 0.0, ...
    "direction", 1.0, ...
    "absDistance", 0.0, ...
    "peakVelocity", 0.0, ...
    "acceleration", 0.0, ...
    "accelTime", 0.0, ...
    "cruiseTime", 0.0, ...
    "duration", 0.0, ...
    "valid", false);
end

function profile = configureTrapezoidAxis(distance, maxVelocity, maxAcceleration, duration)
profile = emptyAxisProfile();
profile.distance = distance;
profile.direction = signOrPositive(distance);
profile.absDistance = abs(distance);
profile.duration = duration;

if profile.absDistance <= 1e-6
    profile.valid = true;
    return;
end
if duration <= 1e-6
    error("makeJointTrapezoidTrajectory:InvalidDuration", ...
        "Nonzero distance cannot use zero synchronized duration.");
end

maxAcc = abs(maxAcceleration);
maxVel = abs(maxVelocity);
disc = duration * duration - 4.0 * profile.absDistance / maxAcc;
if disc < 0.0
    if disc < -1e-5
        error("makeJointTrapezoidTrajectory:InvalidSync", ...
            "Synchronized duration is shorter than the axis minimum duration.");
    end
    disc = 0.0;
end

ta = 0.5 * (duration - sqrt(disc));
ta = min(max(ta, 0.0), 0.5 * duration);
vPeak = maxAcc * ta;
if vPeak > maxVel + 1e-4
    vPeak = maxVel;
    ta = vPeak / maxAcc;
end

profile.accelTime = ta;
profile.cruiseTime = max(duration - 2.0 * ta, 0.0);
profile.peakVelocity = vPeak;
profile.acceleration = double(ta > 1e-6) * maxAcc;
profile.valid = true;
end

function sample = sampleTrapezoidAxis(profile, elapsed)
sample = struct("position", 0.0, "velocity", 0.0, "acceleration", 0.0);
if ~profile.valid
    return;
end
if profile.duration <= 1e-6
    sample.position = profile.distance;
    return;
end

t = min(max(elapsed, 0.0), profile.duration);
posAbs = profile.absDistance;
velAbs = 0.0;
accAbs = 0.0;

if profile.absDistance <= 1e-6 || profile.acceleration <= 1e-6
    posAbs = profile.absDistance;
elseif t <= profile.accelTime
    posAbs = 0.5 * profile.acceleration * t * t;
    velAbs = profile.acceleration * t;
    accAbs = profile.acceleration;
elseif t <= profile.accelTime + profile.cruiseTime
    cruiseT = t - profile.accelTime;
    posAbs = 0.5 * profile.acceleration * profile.accelTime^2 + ...
        profile.peakVelocity * cruiseT;
    velAbs = profile.peakVelocity;
elseif t < profile.duration
    decelT = t - profile.accelTime - profile.cruiseTime;
    beforeDecel = 0.5 * profile.acceleration * profile.accelTime^2 + ...
        profile.peakVelocity * profile.cruiseTime;
    posAbs = beforeDecel + profile.peakVelocity * decelT - ...
        0.5 * profile.acceleration * decelT^2;
    velAbs = profile.peakVelocity - profile.acceleration * decelT;
    accAbs = -profile.acceleration;
end

if elapsed >= profile.duration
    sample.position = profile.distance;
else
    sample.position = profile.direction * posAbs;
    sample.velocity = profile.direction * velAbs;
    sample.acceleration = profile.direction * accAbs;
end
end

function value = signOrPositive(x)
if x >= 0
    value = 1.0;
else
    value = -1.0;
end
end

