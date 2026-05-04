function [q, info] = solveIK(modelSpec, targetTform, seed, solverSpec)
%SOLVEIK Dispatch IK to a named solver implementation.

if nargin < 4 || isempty(solverSpec)
    solverSpec = defaultSolverSpec("numeric");
end

switch lower(string(solverSpec.name))
    case "numeric"
        [q, info] = solveIK_numeric(modelSpec, targetTform, seed, solverSpec);
    case {"pieper", "pieper6r", "pieper_6r"}
        [q, info] = solveIK_pieper(modelSpec, targetTform, seed, solverSpec);
    case {"analytic", "analytic_builtin"}
        [q, info] = solveIK_analytic(modelSpec, targetTform, seed, solverSpec);
    case {"3pit", "threepit"}
        solverSpec = withIkMode(solverSpec, 0);
        [q, info] = solveIK_3pit(modelSpec, targetTform, seed, solverSpec);
    case {"3pit_analytic", "threepit_analytic"}
        solverSpec = withIkMode(solverSpec, 1);
        [q, info] = solveIK_3pit(modelSpec, targetTform, seed, solverSpec);
    case {"3pit_hybrid", "threepit_hybrid", "3pit_fused", "threepit_fused"}
        solverSpec = withIkMode(solverSpec, 2);
        [q, info] = solveIK_3pit(modelSpec, targetTform, seed, solverSpec);
    otherwise
        error("solveIK:UnknownSolver", "Unknown IK solver: %s", solverSpec.name);
end
end

function solverSpec = withIkMode(solverSpec, mode)
if ~isfield(solverSpec, "ikMode") || isempty(solverSpec.ikMode)
    solverSpec.ikMode = mode;
end
end
