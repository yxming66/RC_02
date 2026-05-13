function modelSpec = model_6dof_pieper()
%MODEL_6DOF_PIEPER Pieper-friendly 6DOF URDF model spec.

root = armSimRoot();
urdfPath = fullfile(root, "..", "arm_model", ...
    "机械臂URDF六坐标系不带夹爪", "pieper_params", "urdf", "pieper_params.urdf");

modelSpec = struct();
modelSpec.name = "6dof_pieper_params";
modelSpec.kind = "6dof";
modelSpec.dof = 6;
modelSpec.urdfPath = urdfPath;
modelSpec.robot = [];
modelSpec.dataFormat = "row";
modelSpec.baseName = "base_link";
modelSpec.eeName = "Link6";
modelSpec.home = zeros(1, 6);
modelSpec.jointNames = ["j1", "j2", "j3", "j4", "j5", "j6"];
modelSpec.jointLimits = [
    -15.7, 15.7;
    -1.57, 1.57;
    -1.0,  3.0;
     0.0,  6.3;
    -1.9,  1.9;
     0.0,  6.3;
];
modelSpec.tcp = eye(4);
modelSpec.notes = "Uses arm_model/pieper_params for preliminary Pieper IK validation.";
end

