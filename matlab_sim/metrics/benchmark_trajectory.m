function summary = benchmark_trajectory(modelSpec, poses, seed, solverSpec)
%BENCHMARK_TRAJECTORY Evaluate IK continuity over a pose trajectory.

modelSpec = loadRobotModel(modelSpec);
seed = normalizeJointVector(seed, modelSpec.dof);
n = size(poses, 1);

cfg = default_benchmark_config(modelSpec);
cfg.sampleCount = n;
cfg.solver = solverSpec;

template = struct( ...
    "success", false, ...
    "positionError", inf, ...
    "orientationError", inf, ...
    "solveTime", NaN, ...
    "jointJump", NaN, ...
    "q", nan(1, modelSpec.dof), ...
    "pose", nan(1, 6), ...
    "status", "unknown");
results = repmat(template, n, 1);

lastQ = seed;
for i = 1:n
    targetT = poseToTform(poses(i, :));
    [q, info] = solveIK(modelSpec, targetT, lastQ, solverSpec);
    check = checkIkSolution(modelSpec, q, targetT, solverSpec);

    results(i).success = check.success;
    results(i).positionError = check.positionError;
    results(i).orientationError = check.orientationError;
    results(i).solveTime = info.solveTime;
    results(i).jointJump = norm(q - lastQ);
    results(i).q = q;
    results(i).pose = poses(i, :);
    results(i).status = string(info.status);

    if check.success
        lastQ = q;
    end
end

summary = summarizeBenchmark(results);
end

