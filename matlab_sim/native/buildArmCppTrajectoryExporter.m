function exePath = buildArmCppTrajectoryExporter(forceRebuild)
%BUILDARMCPPTRAJECTORYEXPORTER Build the C++ 3pit trajectory exporter executable.

if nargin < 1
    forceRebuild = false;
end

root = armSimRoot();
repoRoot = fileparts(root);
nativeDir = fullfile(root, "native");
buildDir = fullfile(root, "build", "native");
if ~exist(buildDir, "dir")
    mkdir(buildDir);
end

exePath = fullfile(buildDir, "arm_cpp_trajectory_export.exe");
sourcePath = fullfile(nativeDir, "arm_cpp_trajectory_export.cpp");
if ~forceRebuild && isfile(exePath)
    return;
end

includes = [
    "-I" + quotePath(repoRoot)
    "-I" + quotePath(fullfile(repoRoot, "User"))
    "-I" + quotePath(fullfile(repoRoot, "User", "component"))
    "-I" + quotePath(fullfile(repoRoot, "User", "component", "toolbox"))
    "-I" + quotePath(fullfile(repoRoot, "Drivers", "CMSIS", "DSP", "Include"))
    "-I" + quotePath(fullfile(repoRoot, "Drivers", "CMSIS", "Core", "Include"))
    "-I" + quotePath(fullfile(repoRoot, "Drivers", "CMSIS", "Include"))
];

defines = ["-DARM_MATH_CM4", "-D__FPU_PRESENT=1", "-DARM_MATH_MATRIX_CHECK"];
sources = [
    string(sourcePath)
    string(fullfile(repoRoot, "User", "robotics", "arm", "tests", "host_cmsis_matrix_stub.cpp"))
    string(fullfile(repoRoot, "User", "component", "toolbox", "matrix.cpp"))
    string(fullfile(repoRoot, "User", "component", "toolbox", "utils.cpp"))
    string(fullfile(repoRoot, "User", "component", "toolbox", "robotics.cpp"))
    string(fullfile(repoRoot, "User", "robotics", "arm", "adapter", "toolbox_adapter.cpp"))
    string(fullfile(repoRoot, "User", "robotics", "arm", "model", "joint.cpp"))
    string(fullfile(repoRoot, "User", "robotics", "arm", "model", "link.cpp"))
    string(fullfile(repoRoot, "User", "robotics", "arm", "model", "tool_frame.cpp"))
    string(fullfile(repoRoot, "User", "robotics", "arm", "model", "serial_chain.cpp"))
    string(fullfile(repoRoot, "User", "robotics", "arm", "kinematics", "pose_error.cpp"))
    string(fullfile(repoRoot, "User", "robotics", "arm", "kinematics", "jacobian.cpp"))
    string(fullfile(repoRoot, "User", "robotics", "arm", "kinematics", "analytic_solvers", "ik_3r_planar.cpp"))
    string(fullfile(repoRoot, "User", "robotics", "arm", "kinematics", "analytic_solvers", "ik_6r_pieper.cpp"))
];

sourceArgs = strings(size(sources));
for i = 1:numel(sources)
    sourceArgs(i) = quotePath(sources(i));
end

cmd = strjoin(["g++", "-std=c++17", "-O0", "-g", "-fpermissive", includes(:).', defines, ...
    sourceArgs(:).', "-o", quotePath(exePath)], " ");
[status, output] = system(cmd);
if status ~= 0
    error("buildArmCppTrajectoryExporter:BuildFailed", ...
        "Failed to build C++ trajectory exporter:\n%s", output);
end
end

function out = quotePath(pathValue)
out = """" + string(pathValue) + """";
end
