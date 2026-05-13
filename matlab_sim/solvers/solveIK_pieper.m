function [q, info] = solveIK_pieper(modelSpec, targetTform, seed, solverSpec)
%SOLVEIK_PIEPER Placeholder dispatch for a Pieper / spherical-wrist solver.

% The simulation framework is ready for a project-specific Pieper solver.
% Until that formula is filled in, use the numeric baseline so demos and
% benchmarks remain executable.
[q, info] = solveIK_numeric(modelSpec, targetTform, seed, solverSpec);
info.name = "pieper";
info.note = "Fallback to numeric baseline; replace with Pieper branch enumeration.";
end

