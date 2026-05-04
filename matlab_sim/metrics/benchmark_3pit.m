function summary = benchmark_3pit(cfg)
%BENCHMARK_3PIT Run yz/pitch target benchmark for a 3pit model.

cfg.model = loadRobotModel(cfg.model);
cfg.solver.positionTolerance = cfg.positionTolerance;
cfg.solver.orientationTolerance = cfg.orientationTolerance;
cfg.solver.targetMask = cfg.model.targetMask;
cfg.solver.maxVelocity = cfg.maxVelocity;
cfg.solver.maxAcceleration = cfg.maxAcceleration;
cfg.solver.dt = cfg.dt;
cfg.solver.oneShotMaxJointStep = 0.0;
poses = sample3pitTargets(cfg);
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

    jointJump = norm(q - lastQ);
    results(i).success = check.success && jointJump <= cfg.maxJointJump;
    results(i).positionError = check.positionError;
    results(i).orientationError = check.orientationError;
    results(i).solveTime = info.solveTime;
    results(i).jointJump = jointJump;
    results(i).q = q;
    results(i).pose = poses(i, :);
    results(i).status = string(info.status);

    if results(i).success
        lastQ = q;
    end
end

summary = summarizeBenchmark(results);
end
