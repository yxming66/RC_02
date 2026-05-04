function solverSpec = defaultSolverSpec(name)
%DEFAULTSOLVERSPEC Create a solver configuration.

if nargin < 1 || strlength(string(name)) == 0
    name = "numeric";
end

solverSpec = struct();
solverSpec.name = string(name);
solverSpec.weights = [1 1 1 1 1 1];
solverSpec.maxIterations = 200;
solverSpec.allowRandomRestart = false;
solverSpec.positionTolerance = 1e-3;
solverSpec.orientationTolerance = 1e-2;
solverSpec.timeout = inf;
solverSpec.branchSelection = "closest_to_seed";

if any(lower(string(name)) == ["3pit", "threepit", ...
        "3pit_analytic", "threepit_analytic", ...
        "3pit_hybrid", "threepit_hybrid", "3pit_fused", "threepit_fused"])
    solverSpec.weights = [0.02, 1.0, 1.0, 0.02, 1.0, 0.02];
    solverSpec.targetMask = [false, true, true, false, true, false];
    solverSpec.maxIterations = 120;
    solverSpec.positionTolerance = 3e-3;
    solverSpec.orientationTolerance = 2e-2;
    solverSpec.dt = 0.02;
    solverSpec.maxVelocity = [1.2, 1.0, 1.0];
    solverSpec.maxAcceleration = [3.0, 2.5, 2.5];
    solverSpec.maxLinearVelocity = 0.08;
    solverSpec.maxAngularVelocity = 0.8;
    solverSpec.maxLinearAcceleration = 0.5;
    solverSpec.maxAngularAcceleration = 2.0;
    solverSpec.oneShotMaxJointStep = 0.0;
    solverSpec.controlMaxJointStep = 0.08;
    solverSpec.ikMode = ikModeForName(name);
end
end

function mode = ikModeForName(name)
switch lower(string(name))
    case {"3pit_analytic", "threepit_analytic"}
        mode = 1;
    case {"3pit_hybrid", "threepit_hybrid", "3pit_fused", "threepit_fused"}
        mode = 2;
    otherwise
        mode = 0;
end
end
