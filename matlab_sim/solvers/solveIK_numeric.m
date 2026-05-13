function [q, info] = solveIK_numeric(modelSpec, targetTform, seed, solverSpec)
%SOLVEIK_NUMERIC MATLAB inverseKinematics baseline.

modelSpec = loadRobotModel(modelSpec);
seed = normalizeJointVector(seed, modelSpec.dof);

tic;
ik = inverseKinematics("RigidBodyTree", modelSpec.robot);
ik.SolverParameters.MaxIterations = solverSpec.maxIterations;

targetForRobot = targetTform;
if isfield(modelSpec, "tcp") && ~isempty(modelSpec.tcp)
    targetForRobot = targetTform / modelSpec.tcp;
end

[q, solInfo] = ik(modelSpec.eeName, targetForRobot, solverSpec.weights, seed);
elapsed = toc;

q = normalizeJointVector(q, modelSpec.dof);
check = checkIkSolution(modelSpec, q, targetTform, solverSpec);

info = makeSolverInfo("numeric");
info.success = check.success;
info.status = string(solInfo.Status);
info.solveTime = elapsed;
info.positionError = check.positionError;
info.orientationError = check.orientationError;
info.iterations = getfield_or(solInfo, "Iterations", NaN);
info.exitFlag = getfield_or(solInfo, "ExitFlag", NaN);
info.raw = solInfo;
end

