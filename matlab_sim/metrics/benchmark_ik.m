function summary = benchmark_ik(cfg)
%BENCHMARK_IK Run random target IK benchmark.

cfg.model = loadRobotModel(cfg.model);
poses = sampleTargetPoses(cfg);
seed = normalizeJointVector(cfg.seed, cfg.model.dof);

template = struct( ...
    "success", false, ...
    "positionError", inf, ...
    "orientationError", inf, ...
    "solveTime", NaN, ...
    "jointJump", NaN, ...
    "q", nan(1, cfg.model.dof), ...
    "pose", nan(1, 6), ...
    "status", "unknown");
results = repmat(template, cfg.sampleCount, 1);

lastQ = seed;
for i = 1:cfg.sampleCount
    targetT = poseToTform(poses(i, :));
    [q, info] = solveIK(cfg.model, targetT, lastQ, cfg.solver);
    check = checkIkSolution(cfg.model, q, targetT, cfg.solver);
    jump = norm(q - lastQ);

    results(i).success = check.success;
    results(i).positionError = check.positionError;
    results(i).orientationError = check.orientationError;
    results(i).solveTime = info.solveTime;
    results(i).jointJump = jump;
    results(i).q = q;
    results(i).pose = poses(i, :);
    results(i).status = string(info.status);

    if check.success
        lastQ = q;
    end
end

summary = summarizeBenchmark(results);
end

