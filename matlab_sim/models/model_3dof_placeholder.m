function modelSpec = model_3dof_placeholder()
%MODEL_3DOF_PLACEHOLDER Placeholder spec for future custom 3DOF validation.

modelSpec = struct();
modelSpec.name = "3dof_placeholder";
modelSpec.kind = "3dof";
modelSpec.dof = 3;
modelSpec.urdfPath = "";
modelSpec.robot = [];
modelSpec.dataFormat = "row";
modelSpec.baseName = "";
modelSpec.eeName = "";
modelSpec.home = zeros(1, 3);
modelSpec.jointNames = ["j1", "j2", "j3"];
modelSpec.jointLimits = [
    -pi, pi;
    -pi, pi;
    -pi, pi;
];
modelSpec.tcp = eye(4);
modelSpec.notes = "Fill robot/DH/URDF data before running FK or IK.";
end
