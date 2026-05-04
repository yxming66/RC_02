function out = runArmCppTrajectoryExporter(input, forceRebuild)
%RUNARMCPPTRAJECTORYEXPORTER Run actual C++ 3pit arm code and read CSV.

if nargin < 2
    forceRebuild = true;
end

root = armSimRoot();
outDir = fullfile(root, "build", "native");
if ~exist(outDir, "dir")
    mkdir(outDir);
end

inputPath = fullfile(outDir, "arm_cpp_trajectory_input.csv");
movejPath = fullfile(outDir, "arm_cpp_movej.csv");
cartPath = fullfile(outDir, "arm_cpp_cartesian.csv");
framePath = fullfile(outDir, "arm_cpp_frame_targets.csv");
ikPath = fullfile(outDir, "arm_cpp_ik.csv");
servoPath = fullfile(outDir, "arm_cpp_servo.csv");
writeInput(inputPath, input);

exePath = buildArmCppTrajectoryExporter(forceRebuild);
cmd = strjoin([quotePath(exePath), quotePath(inputPath), ...
    quotePath(movejPath), quotePath(cartPath), quotePath(framePath), ...
    quotePath(ikPath), quotePath(servoPath)], " ");
[status, output] = system(cmd);
if status ~= 0
    error("runArmCppTrajectoryExporter:RunFailed", ...
        "C++ 3pit trajectory exporter failed:\n%s", output);
end

out = struct();
out.inputPath = inputPath;
out.movejPath = movejPath;
out.cartesianPath = cartPath;
out.frameTargetsPath = framePath;
out.ikPath = ikPath;
out.servoPath = servoPath;
out.movej = readmatrix(movejPath, "NumHeaderLines", 1);
out.cartesian = readmatrix(cartPath, "NumHeaderLines", 1);
out.frameTargets = readmatrix(framePath, "NumHeaderLines", 1);
out.cppIk = readmatrix(ikPath, "NumHeaderLines", 1);
out.servo = readmatrix(servoPath, "NumHeaderLines", 1);
end

function writeInput(path, input)
rows = zeros(12, 6);
rows(1, 1:3) = normalize3(input.q0, "q0");
rows(2, 1:3) = normalize3(input.q1, "q1");
rows(3, 1:3) = normalize3(input.maxVelocity, "maxVelocity");
rows(4, 1:3) = normalize3(input.maxAcceleration, "maxAcceleration");
rows(5, 1) = input.movejDt;
rows(6, :) = input.cartStartPose;
rows(7, :) = input.cartGoalPose;
rows(8, 1:4) = [
    input.cartLimits.maxLinearVelocity, ...
    input.cartLimits.maxAngularVelocity, ...
    input.cartLimits.maxLinearAcceleration, ...
    input.cartLimits.maxAngularAcceleration];
rows(9, 1) = input.cartDt;
if isfield(input, "frameDeltaPose") && ~isempty(input.frameDeltaPose)
    rows(10, :) = input.frameDeltaPose;
end
if isfield(input, "ikMode") && ~isempty(input.ikMode)
    rows(11, 1) = input.ikMode;
end
rows(12, 1) = getFieldOrDefault(input, "controlDuration", 0.0);
rows(12, 2) = getFieldOrDefault(input, "maxJointStep", 0.08);
rows(12, 3) = getFieldOrDefault(input, "positionTolerance", 1e-3);
rows(12, 4) = getFieldOrDefault(input, "orientationTolerance", 1e-2);
rows(12, 5) = getFieldOrDefault(input, "controlFrame", 0);
writematrix(rows, path);
end

function values = normalize3(values, name)
values = double(values(:).');
if isscalar(values)
    values = repmat(values, 1, 3);
end
if numel(values) ~= 3
    error("runArmCppTrajectoryExporter:InvalidInput", ...
        "%s must be scalar or a 3-element vector for 3pit simulation.", name);
end
end

function value = getFieldOrDefault(input, fieldName, defaultValue)
if isfield(input, fieldName) && ~isempty(input.(fieldName))
    value = input.(fieldName);
else
    value = defaultValue;
end
end

function out = quotePath(pathValue)
out = """" + string(pathValue) + """";
end
