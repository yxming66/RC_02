function traj = makeCartesianCubicBlendTrajectory(startPose, goalPose, limits, dt)
%MAKECARTESIANCUBICBLENDTRAJECTORY Cubic-blend SE(3) pose trajectory.
%
% The scalar timing mirrors component/trajectory/cubic_blend.hpp. The pose
% interpolation follows the same body-relative transform idea used by the C++
% CartesianTrajectory wrapper.

if nargin < 3 || isempty(limits)
    limits = struct();
end
if nargin < 4 || isempty(dt)
    dt = 0.02;
end

limits = fillLimits(limits);
startPose = double(startPose(:).');
goalPose = double(goalPose(:).');
startT = poseToTform(startPose);
goalT = poseToTform(goalPose);

relativeT = startT \ goalT;
linearDelta = relativeT(1:3, 4).';
rotDelta = relativeT(1:3, 1:3);
[axis, angle] = axisAngleFromRotation(rotDelta);
rotvec = axis .* angle;

linearDistance = norm(linearDelta);
angularDistance = abs(angle);
linearDuration = cubicDurationFromLimits( ...
    linearDistance, limits.maxLinearVelocity, limits.maxLinearAcceleration);
angularDuration = cubicDurationFromLimits( ...
    angularDistance, limits.maxAngularVelocity, limits.maxAngularAcceleration);
duration = max(linearDuration, angularDuration);

if duration <= 1e-6
    time = 0.0;
else
    time = 0:dt:duration;
    if time(end) < duration
        time(end + 1) = duration; %#ok<AGROW>
    end
end

n = numel(time);
poses = zeros(n, 6);
tforms = zeros(4, 4, n);
bodyTwist = zeros(n, 6);
bodyAcceleration = zeros(n, 6);

for k = 1:n
    blend = sampleCubicBlend(time(k), duration);
    stepT = eye(4);
    stepT(1:3, 1:3) = rotationFromAxisAngle(axis, angle * blend.s);
    stepT(1:3, 4) = (linearDelta * blend.s).';
    T = startT * stepT;
    tforms(:, :, k) = T;
    poses(k, :) = tformToPose(T);
    bodyTwist(k, :) = [linearDelta, rotvec] .* blend.ds;
    bodyAcceleration(k, :) = [linearDelta, rotvec] .* blend.dds;
end

poses(end, :) = goalPose;
tforms(:, :, end) = goalT;
bodyTwist(end, :) = 0.0;
bodyAcceleration(end, :) = 0.0;

traj = struct();
traj.type = "cartesian_cubic_blend";
traj.time = time(:);
traj.poses = poses;
traj.tforms = tforms;
traj.bodyTwist = bodyTwist;
traj.bodyAcceleration = bodyAcceleration;
traj.duration = duration;
traj.linearDistance = linearDistance;
traj.angularDistance = angularDistance;
traj.limits = limits;
traj.valid = true;
end

function limits = fillLimits(limits)
limits = setDefault(limits, "maxLinearVelocity", 0.2);
limits = setDefault(limits, "maxAngularVelocity", 1.0);
limits = setDefault(limits, "maxLinearAcceleration", 0.5);
limits = setDefault(limits, "maxAngularAcceleration", 2.0);
end

function s = setDefault(s, name, value)
if ~isfield(s, name) || isempty(s.(name))
    s.(name) = value;
end
end

function duration = cubicDurationFromLimits(distance, maxVelocity, maxAcceleration)
distance = abs(distance);
if distance <= 1e-6
    duration = 0.0;
    return;
end
if maxVelocity <= 1e-6 || maxAcceleration <= 1e-6
    duration = -1.0;
    return;
end
velocityBound = 1.5 * distance / maxVelocity;
accelerationBound = sqrt(6.0 * distance / maxAcceleration);
duration = max(velocityBound, accelerationBound);
end

function [axis, angle] = axisAngleFromRotation(R)
angle = acos(min(max((trace(R) - 1.0) * 0.5, -1.0), 1.0));
if abs(angle) <= 1e-9
    axis = [1.0, 0.0, 0.0];
    angle = 0.0;
    return;
end
axis = [
    R(3, 2) - R(2, 3), ...
    R(1, 3) - R(3, 1), ...
    R(2, 1) - R(1, 2)] ./ (2.0 * sin(angle));
axisNorm = norm(axis);
if axisNorm <= 1e-9
    axis = [1.0, 0.0, 0.0];
else
    axis = axis ./ axisNorm;
end
end

function R = rotationFromAxisAngle(axis, angle)
axis = axis(:);
if abs(angle) <= 1e-12 || norm(axis) <= 1e-12
    R = eye(3);
    return;
end
axis = axis ./ norm(axis);
x = axis(1);
y = axis(2);
z = axis(3);
c = cos(angle);
s = sin(angle);
C = 1.0 - c;
R = [
    x*x*C + c,   x*y*C - z*s, x*z*C + y*s;
    y*x*C + z*s, y*y*C + c,   y*z*C - x*s;
    z*x*C - y*s, z*y*C + x*s, z*z*C + c];
end

