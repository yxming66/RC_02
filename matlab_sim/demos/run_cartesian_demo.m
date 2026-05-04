function out = run_cartesian_demo(modelSpec, solverName)
%RUN_CARTESIAN_DEMO Cartesian IK demo.

if nargin < 1
    modelSpec = model_3dof_3pit();
end
if nargin < 2
    solverName = "3pit";
end

modelSpec = loadRobotModel(modelSpec);
seed = modelSpec.controlHome;
homePose = tformToPose(fkArm(modelSpec, seed));
targetPose = homePose;
targetPose(2) = targetPose(2) + 0.015;
targetPose(3) = targetPose(3) + 0.015;
targetPose(5) = targetPose(5) + 0.02;

solverSpec = defaultSolverSpec(solverName);
out = cartesianControlStep(modelSpec, targetPose, seed, solverSpec);

fprintf("Cartesian demo model: %s\n", modelSpec.name);
fprintf("Solver: %s\n", solverName);
fprintf("Success: %d, position error: %.6g, orientation error: %.6g\n", ...
    out.success, out.check.positionError, out.check.orientationError);

showArm(modelSpec, out.q);
end
