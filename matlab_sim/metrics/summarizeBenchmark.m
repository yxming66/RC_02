function summary = summarizeBenchmark(results)
%SUMMARIZEBENCHMARK Aggregate IK benchmark results.

success = [results.success];
positionError = [results.positionError];
orientationError = [results.orientationError];
solveTime = [results.solveTime];
jointJump = [results.jointJump];

summary = struct();
summary.count = numel(results);
summary.successCount = nnz(success);
summary.successRate = nnz(success) / max(1, numel(results));
summary.meanPositionError = mean(positionError, "omitnan");
summary.maxPositionError = max(positionError);
summary.meanOrientationError = mean(orientationError, "omitnan");
summary.maxOrientationError = max(orientationError);
summary.meanSolveTime = mean(solveTime, "omitnan");
summary.maxSolveTime = max(solveTime);
summary.maxJointJump = max(jointJump);
summary.results = results;
end

