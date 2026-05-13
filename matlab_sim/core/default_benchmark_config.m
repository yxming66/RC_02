function cfg = default_benchmark_config(modelSpec)
%DEFAULT_BENCHMARK_CONFIG Default IK benchmark settings.

cfg = struct();
cfg.model = modelSpec;
cfg.model = loadRobotModel(cfg.model);
cfg.solver = defaultSolverSpec("numeric");
cfg.sampleCount = 100;
cfg.seed = cfg.model.home;
cfg.positionBounds = [-0.25 0.45; -0.35 0.35; 0.05 0.65];
cfg.rpyBounds = [-pi pi; -pi/2 pi/2; -pi pi];
cfg.positionTolerance = cfg.solver.positionTolerance;
cfg.orientationTolerance = cfg.solver.orientationTolerance;
cfg.maxJointJump = 0.75;
cfg.rngSeed = 1;
end

