function run_joint_demo(modelSpec)
%RUN_JOINT_DEMO Joint-space FK visualization demo.

if nargin < 1
    modelSpec = model_3dof_3pit();
end

modelSpec = loadRobotModel(modelSpec);
q = modelSpec.controlHome;
out = jointControlStep(modelSpec, q);

fprintf("Joint demo model: %s\n", modelSpec.name);
fprintf("Pose [x y z roll pitch yaw]:\n");
disp(out.pose);

showArm(modelSpec, q);
end
