function modelSpec = run_3dof_placeholder_demo()
%RUN_3DOF_PLACEHOLDER_DEMO Show how a custom 3DOF model will plug in.

modelSpec = model_3dof_placeholder();
solverSpec = defaultSolverSpec("3pit");

fprintf("3DOF placeholder model created: %s\n", modelSpec.name);
fprintf("Solver placeholder: %s\n", solverSpec.name);
fprintf("Fill modelSpec.robot or modelSpec.urdfPath, then implement solvers/solveIK_3pit.m.\n");
end
