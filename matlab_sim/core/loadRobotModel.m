function modelSpec = loadRobotModel(modelSpec)
%LOADROBOTMODEL Load a modelSpec into a MATLAB rigidBodyTree when needed.

if isfield(modelSpec, "robot") && ~isempty(modelSpec.robot)
    return;
end

if ~isfield(modelSpec, "urdfPath") || strlength(string(modelSpec.urdfPath)) == 0
    error("loadRobotModel:MissingModel", ...
        "modelSpec.robot is empty and modelSpec.urdfPath is not set.");
end

if ~isfile(modelSpec.urdfPath)
    error("loadRobotModel:MissingUrdf", ...
        "URDF file does not exist: %s", modelSpec.urdfPath);
end

robot = importrobot(modelSpec.urdfPath);
robot.DataFormat = char(modelSpec.dataFormat);
robot.Gravity = [0 0 -9.81];
modelSpec.robot = robot;

if ~isfield(modelSpec, "home") || isempty(modelSpec.home)
    modelSpec.home = homeConfiguration(robot);
end
end

