function [q, info] = solveIK_analytic(modelSpec, targetTform, seed, solverSpec)
%SOLVEIK_ANALYTIC Placeholder for closed-form analytic solvers.

% MATLAB's generic rigidBodyTree does not expose a project-specific analytic
% solver automatically. Keep this dispatch point so closed-form solvers can
% be added without changing controls or benchmarks.
[q, info] = solveIK_numeric(modelSpec, targetTform, seed, solverSpec);
info.name = "analytic_builtin";
info.note = "Fallback to numeric baseline; add a closed-form analytic solver here.";
end

