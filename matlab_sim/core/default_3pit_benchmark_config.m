function cfg = default_3pit_benchmark_config(modelSpec)
%DEFAULT_3PIT_BENCHMARK_CONFIG Default yz/pitch benchmark settings.

cfg = struct();
cfg.model = loadRobotModel(modelSpec);
cfg.solver = defaultSolverSpec("3pit");
cfg.sampleCount = 30;
cfg.seed = cfg.model.controlHome;
cfg.nominalPose = tformToPose(fkArm(cfg.model, cfg.seed));
cfg.yBounds = [cfg.nominalPose(2) - 0.04, cfg.nominalPose(2) + 0.04];
cfg.zBounds = [cfg.nominalPose(3) - 0.04, cfg.nominalPose(3) + 0.04];
cfg.pitchBounds = [cfg.nominalPose(5) - 0.12, cfg.nominalPose(5) + 0.12];
cfg.positionTolerance = 3e-3;
cfg.orientationTolerance = 2e-2;
cfg.maxJointJump = 0.5;
cfg.maxVelocity = cfg.model.maxVelocity;
cfg.maxAcceleration = cfg.model.maxAcceleration;
cfg.maxJointStep = cfg.model.maxJointStep;
cfg.dt = cfg.model.dt;
cfg.rngSeed = 2;
end
