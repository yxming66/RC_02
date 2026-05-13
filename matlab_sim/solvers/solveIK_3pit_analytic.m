function [q, info] = solveIK_3pit_analytic(modelSpec, targetTform, seed, solverSpec)
%SOLVEIK_3PIT_ANALYTIC Geometric yz/pitch IK for the 3pit arm.

if modelSpec.dof ~= 3
    error("solveIK_3pit_analytic:InvalidModel", ...
        "3pit analytic solver expects modelSpec.dof == 3.");
end

modelSpec = loadRobotModel(modelSpec);
seed = normalizeJointVector(seed, modelSpec.dof);
targetPose = tformToPose(targetTform);

geom = get3pitGeometry(modelSpec);
phi = targetPose(5);
yLocal = -(targetPose(2) - geom.y0) - geom.l3 * sin(phi);
zLocal = targetPose(3) - geom.zBase - geom.l3 * cos(phi);

den = 2.0 * geom.l1 * geom.l2;
c2 = (yLocal^2 + zLocal^2 - geom.l1^2 - geom.l2^2) / den;

info = makeSolverInfo("3pit_analytic");
info.status = "unreachable";
info.solveTime = NaN;
info.iterations = 1;
info.raw = struct("c2", c2, "targetPose", targetPose, "geometry", geom);

if ~isfinite(c2) || c2 < -1.0 - 1e-8 || c2 > 1.0 + 1e-8
    q = seed;
    return;
end

c2 = min(max(c2, -1.0), 1.0);
s2Abs = sqrt(max(0.0, 1.0 - c2^2));
candidates = zeros(2, 3);
for i = 1:2
    s2 = s2Abs;
    if i == 2
        s2 = -s2Abs;
    end
    q2 = atan2(s2, c2);
    q1 = atan2(yLocal, zLocal) - atan2(geom.l2 * s2, ...
        geom.l1 + geom.l2 * c2);
    q3 = phi - q1 - q2;
    candidates(i, :) = wrapToPiLocal([q1, q2, q3]);
end

best = struct("valid", false, "score", inf, "q", seed, "check", []);
for i = 1:size(candidates, 1)
    qCandidate = alignToSeed(candidates(i, :), seed);
    if isfield(modelSpec, "jointLimits") && ~jointWithinLimits(modelSpec, qCandidate)
        continue;
    end
    check = checkIkSolution(modelSpec, qCandidate, targetTform, solverSpec);
    taskScore = check.positionError + check.orientationError + ...
        1e-3 * norm(qCandidate - seed);
    if check.withinLimits && taskScore < best.score
        best.valid = true;
        best.score = taskScore;
        best.q = qCandidate;
        best.check = check;
    end
end

q = best.q;
if best.valid
    info.success = best.check.success;
    if best.check.success
        info.status = "success";
    else
        info.status = "residual_above_tolerance";
    end
    info.positionError = best.check.positionError;
    info.orientationError = best.check.orientationError;
    info.fullPositionError = best.check.fullPositionError;
    info.fullOrientationError = best.check.fullOrientationError;
else
    info.status = "limit_or_residual_failure";
end
info.raw.candidates = candidates;
end

function geom = get3pitGeometry(modelSpec)
q0 = zeros(1, modelSpec.dof);
homePose = tformToPose(fkArm(modelSpec, q0));
geom = struct();
geom.l1 = 0.35;
geom.l2 = 0.25;
geom.l3 = 0.14696;
geom.y0 = homePose(2);
geom.zBase = homePose(3) - geom.l1 - geom.l2 - geom.l3;
end

function q = alignToSeed(q, seed)
for i = 1:numel(q)
    q(i) = seed(i) + wrapToPiLocal(q(i) - seed(i));
end
end

function angles = wrapToPiLocal(angles)
angles = mod(angles + pi, 2 * pi) - pi;
end
