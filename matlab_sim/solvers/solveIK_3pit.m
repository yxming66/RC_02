function [q, info] = solveIK_3pit(modelSpec, targetTform, seed, solverSpec)
%SOLVEIK_3PIT C++ arm algorithm backed yz/pitch IK for the 3pit model.

if modelSpec.dof ~= 3
    error("solveIK_3pit:InvalidModel", "3pit solver expects modelSpec.dof == 3.");
end

modelSpec = loadRobotModel(modelSpec);
seed = normalizeJointVector(seed, modelSpec.dof);
targetPose = tformToPose(targetTform);
currentPose = tformToPose(fkArm(modelSpec, seed));

input = makeCppInput(modelSpec, seed, currentPose, targetPose, solverSpec);

persistent nativeBuilt;
if isempty(nativeBuilt)
    nativeBuilt = false;
end

try
    out = runArmCppTrajectoryExporter(input, ~nativeBuilt);
    nativeBuilt = true;
    row = out.cppIk(out.cppIk(:, 1) == 2, :);
    if isempty(row)
        error("solveIK_3pit:MissingCppRow", ...
            "C++ IK output did not contain the absolute-goal row.");
    end
    row = row(1, :);
    qCpp = normalizeJointVector(row(17:19), modelSpec.dof);
    q = qCpp;

    info = makeSolverInfo("3pit_cpp");
    info.success = row(2) ~= 0;
    info.status = statusName(row(3));
    info.solveTime = NaN;
    info.positionError = row(10);
    info.orientationError = row(11);
    info.fullPositionError = row(8);
    info.fullOrientationError = row(9);
    info.iterations = row(6);
    info.exitFlag = row(3);
    info.failStage = row(4);
    info.failureReason = row(5);
    info.finalError = row(7);
    info.cppPositionError = row(8);
    info.cppOrientationError = row(9);
    info.cppTaskPositionError = row(10);
    info.cppTaskOrientationError = row(11);
    info.singularity = row(12);
    info.limitViolation = row(13);
    info.usedAnalytic = row(14) ~= 0;
    info.usedNumericRefine = row(15) ~= 0;
    info.usedNumericFallback = row(16) ~= 0;
    info.raw = out;
    return;
catch nativeError
    q = seed;
    info = makeSolverInfo("3pit_cpp");
    info.success = false;
    info.status = "native_error";
    info.solveTime = NaN;
    info.positionError = inf;
    info.orientationError = inf;
    info.fullPositionError = inf;
    info.fullOrientationError = inf;
    info.iterations = NaN;
    info.exitFlag = NaN;
    info.raw = nativeError;
end
end

function input = makeCppInput(modelSpec, seed, currentPose, targetPose, solverSpec)
maxVelocity = getSpecValue(solverSpec, modelSpec, "maxVelocity", [1.2, 1.0, 1.0]);
maxAcceleration = getSpecValue(solverSpec, modelSpec, "maxAcceleration", [3.0, 2.5, 2.5]);
positionTolerance = getSpecValue(solverSpec, modelSpec, "positionTolerance", 1e-3);
orientationTolerance = getSpecValue(solverSpec, modelSpec, "orientationTolerance", 1e-2);

input = struct();
input.q0 = seed;
input.q1 = seed;
input.maxVelocity = maxVelocity;
input.maxAcceleration = maxAcceleration;
input.movejDt = 0.02;
input.cartStartPose = currentPose;
input.cartGoalPose = targetPose;
input.cartLimits = struct( ...
    "maxLinearVelocity", getSpecValue(solverSpec, modelSpec, "maxLinearVelocity", 0.12), ...
    "maxAngularVelocity", getSpecValue(solverSpec, modelSpec, "maxAngularVelocity", 0.8), ...
    "maxLinearAcceleration", getSpecValue(solverSpec, modelSpec, "maxLinearAcceleration", 0.5), ...
    "maxAngularAcceleration", getSpecValue(solverSpec, modelSpec, "maxAngularAcceleration", 2.0));
input.cartDt = getSpecValue(solverSpec, modelSpec, "dt", 0.02);
input.frameDeltaPose = zeros(1, 6);
input.ikMode = getSpecValue(solverSpec, modelSpec, "ikMode", 0);
input.controlDuration = -1.0;
input.maxJointStep = getSpecValue(solverSpec, modelSpec, "oneShotMaxJointStep", 0.0);
input.positionTolerance = positionTolerance;
input.orientationTolerance = orientationTolerance;
input.controlFrame = 0;
end

function value = getSpecValue(solverSpec, modelSpec, fieldName, defaultValue)
if isfield(solverSpec, fieldName) && ~isempty(solverSpec.(fieldName))
    value = solverSpec.(fieldName);
elseif isfield(modelSpec, fieldName) && ~isempty(modelSpec.(fieldName))
    value = modelSpec.(fieldName);
else
    value = defaultValue;
end
end

function name = statusName(value)
names = [ ...
    "success", ...
    "invalid_target", ...
    "invalid_seed", ...
    "invalid_model", ...
    "unreachable", ...
    "constraint_conflict", ...
    "limit_violation", ...
    "singular", ...
    "max_iterations", ...
    "numerical_failure", ...
    "no_convergence", ...
    "unsupported"];
index = round(value) + 1;
if index >= 1 && index <= numel(names)
    name = names(index);
else
    name = "unknown";
end
end
