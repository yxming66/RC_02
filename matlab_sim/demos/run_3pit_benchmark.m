function summary = run_3pit_benchmark(sampleCount)
%RUN_3PIT_BENCHMARK Run a yz/pitch benchmark for the 3pit model.

if nargin < 1
    sampleCount = 30;
end

modelSpec = model_3dof_3pit();
cfg = default_3pit_benchmark_config(modelSpec);
cfg.sampleCount = sampleCount;
summary = benchmark_3pit(cfg);

fprintf("3pit benchmark model: %s\n", cfg.model.name);
fprintf("Solver: %s\n", cfg.solver.name);
fprintf("Success rate: %.2f%% (%d/%d)\n", ...
    100 * summary.successRate, summary.successCount, summary.count);
fprintf("Mean position error: %.6g, max: %.6g\n", ...
    summary.meanPositionError, summary.maxPositionError);
fprintf("Mean orientation error: %.6g, max: %.6g\n", ...
    summary.meanOrientationError, summary.maxOrientationError);
fprintf("Max joint jump: %.6g rad\n", summary.maxJointJump);
end
