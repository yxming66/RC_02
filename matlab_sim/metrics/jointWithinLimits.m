function ok = jointWithinLimits(modelSpec, q)
%JOINTWITHINLIMITS Check q against modelSpec.jointLimits.

q = normalizeJointVector(q, modelSpec.dof);
if ~isfield(modelSpec, "jointLimits") || isempty(modelSpec.jointLimits)
    ok = true;
    return;
end

limits = modelSpec.jointLimits;
tol = 1e-6;
ok = all(q(:) >= limits(:, 1) - tol) && all(q(:) <= limits(:, 2) + tol);
end

