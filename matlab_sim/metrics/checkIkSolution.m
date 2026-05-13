function check = checkIkSolution(modelSpec, q, targetTform, spec)
%CHECKIKSOLUTION FK-backcheck an IK solution.

if nargin < 4 || isempty(spec)
    spec = defaultSolverSpec("numeric");
end

q = normalizeJointVector(q, modelSpec.dof);
actual = fkArm(modelSpec, q);

fullPositionError = norm(tform2trvec(targetTform) - tform2trvec(actual));
relative = actual \ targetTform;
fullOrientationError = abs(rotm2axang(relative(1:3, 1:3)) * [0; 0; 0; 1]);
positionError = fullPositionError;
orientationError = fullOrientationError;

targetMask = getTargetMask(modelSpec, spec);
if ~isempty(targetMask)
    targetPose = tformToPose(targetTform);
    actualPose = tformToPose(actual);
    poseError = targetPose - actualPose;
    poseError(4:6) = wrapToPiLocal(poseError(4:6));

    positionMask = targetMask(1:3);
    orientationMask = targetMask(4:6);
    positionPoseError = poseError(1:3);
    orientationPoseError = poseError(4:6);
    if any(positionMask)
        positionError = norm(positionPoseError(positionMask));
    else
        positionError = 0.0;
    end
    if any(orientationMask)
        orientationError = norm(orientationPoseError(orientationMask));
    else
        orientationError = 0.0;
    end
end
withinLimits = jointWithinLimits(modelSpec, q);

check = struct();
check.success = positionError <= spec.positionTolerance && ...
    orientationError <= spec.orientationTolerance && withinLimits;
check.positionError = positionError;
check.orientationError = orientationError;
check.fullPositionError = fullPositionError;
check.fullOrientationError = fullOrientationError;
check.withinLimits = withinLimits;
check.actualTform = actual;
check.targetPose = tformToPose(targetTform);
check.actualPose = tformToPose(actual);
end

function targetMask = getTargetMask(modelSpec, spec)
targetMask = [];
if isfield(spec, "targetMask") && ~isempty(spec.targetMask)
    targetMask = logical(spec.targetMask);
elseif isfield(modelSpec, "targetMask") && ~isempty(modelSpec.targetMask)
    targetMask = logical(modelSpec.targetMask);
end
if ~isempty(targetMask) && numel(targetMask) ~= 6
    error("checkIkSolution:InvalidTargetMask", ...
        "targetMask must contain 6 logical entries.");
end
end

function angles = wrapToPiLocal(angles)
angles = mod(angles + pi, 2 * pi) - pi;
end
