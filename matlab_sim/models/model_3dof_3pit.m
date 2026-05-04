function modelSpec = model_3dof_3pit()
%MODEL_3DOF_3PIT 3DOF / 3pit URDF model spec.

root = armSimRoot();
urdfPath = fullfile(root, "assets", "urdf", "armurdf_standard_motor", ...
    "urdf", "armurdf_standard_motor_matlab.urdf");

modelSpec = struct();
modelSpec.name = "3dof_3pit_standard_motor";
modelSpec.kind = "3pit";
modelSpec.dof = 3;
modelSpec.urdfPath = urdfPath;
modelSpec.robot = [];
modelSpec.dataFormat = "row";
modelSpec.baseName = "base_link";
modelSpec.eeName = "tool_Link";
modelSpec.home = zeros(1, 3);
modelSpec.controlHome = [0.0, -0.8, 0.9];
modelSpec.jointNames = ["j1", "j2", "j3"];
modelSpec.jointLimits = [
    -1.57, 1.57;
    -2.32, 2.32;
    -1.95, 1.95;
];
modelSpec.tcp = eye(4);
modelSpec.taskSpace = "yz_pitch";
modelSpec.targetMask = [false, true, true, false, true, false];
modelSpec.dt = 0.02;
modelSpec.maxVelocity = [1.2, 1.0, 1.0];
modelSpec.maxAcceleration = [3.0, 2.5, 2.5];
modelSpec.maxJointStep = 0.08;
modelSpec.cartesianLimits = struct( ...
    "maxLinearVelocity", 0.08, ...
    "maxAngularVelocity", 0.8, ...
    "maxLinearAcceleration", 0.5, ...
    "maxAngularAcceleration", 2.0);
modelSpec.notes = "3pit URDF from matlab_sim/assets/urdf/armurdf_standard_motor.";
end
