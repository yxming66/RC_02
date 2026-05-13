function modelSpec = run_3pit_model_demo()
%RUN_3PIT_MODEL_DEMO Load and show the 3pit URDF model.

modelSpec = model_3dof_3pit();
modelSpec = loadRobotModel(modelSpec);

fprintf("3pit model: %s\n", modelSpec.name);
fprintf("URDF: %s\n", modelSpec.urdfPath);
fprintf("Base: %s, end effector: %s\n", modelSpec.baseName, modelSpec.eeName);
fprintf("Joints: %s\n", strjoin(modelSpec.jointNames, ", "));

showArm(modelSpec, modelSpec.home);
end

