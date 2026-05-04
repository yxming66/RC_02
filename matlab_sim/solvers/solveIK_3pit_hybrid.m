function [q, info] = solveIK_3pit_hybrid(modelSpec, targetTform, seed, solverSpec)
%SOLVEIK_3PIT_HYBRID Analytic seed plus C++ numeric refinement.

[qAnalytic, analyticInfo] = solveIK_3pit_analytic( ...
    modelSpec, targetTform, seed, solverSpec);

if analyticInfo.success
    refineSeed = qAnalytic;
else
    refineSeed = seed;
end

[qNumeric, numericInfo] = solveIK_3pit(modelSpec, targetTform, refineSeed, solverSpec);
analyticCheck = checkIkSolution(modelSpec, qAnalytic, targetTform, solverSpec);
numericCheck = checkIkSolution(modelSpec, qNumeric, targetTform, solverSpec);

analyticScore = scoreCheck(analyticCheck, qAnalytic, seed);
numericScore = scoreCheck(numericCheck, qNumeric, seed);

info = makeSolverInfo("3pit_hybrid");
info.raw = struct("analytic", analyticInfo, "numeric", numericInfo);
info.iterations = numericInfo.iterations;
info.solveTime = NaN;

if numericCheck.success && numericScore <= analyticScore
    q = qNumeric;
    info.success = true;
    info.status = "numeric_refined";
    copyCheck();
    return;
end

if analyticCheck.success
    q = qAnalytic;
    info.success = true;
    info.status = "analytic_selected";
    check = analyticCheck;
else
    q = qNumeric;
    info.success = numericCheck.success;
    info.status = "numeric_fallback_" + string(numericInfo.status);
    check = numericCheck;
end

info.positionError = check.positionError;
info.orientationError = check.orientationError;
info.fullPositionError = check.fullPositionError;
info.fullOrientationError = check.fullOrientationError;

    function copyCheck()
        check = numericCheck;
        info.positionError = check.positionError;
        info.orientationError = check.orientationError;
        info.fullPositionError = check.fullPositionError;
        info.fullOrientationError = check.fullOrientationError;
    end
end

function score = scoreCheck(check, q, seed)
score = check.positionError + check.orientationError + 1e-3 * norm(q - seed);
if ~check.withinLimits
    score = score + 1e6;
end
if ~check.success
    score = score + 100.0;
end
end
