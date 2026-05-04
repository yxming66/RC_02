function startup_arm_sim()
%STARTUP_ARM_SIM Add the MATLAB simulation harness to the path.

root = fileparts(mfilename("fullpath"));
addpath(root);
addpath(fullfile(root, "core"));
addpath(fullfile(root, "models"));
addpath(fullfile(root, "solvers"));
addpath(fullfile(root, "trajectories"));
addpath(fullfile(root, "controls"));
addpath(fullfile(root, "metrics"));
addpath(fullfile(root, "demos"));
addpath(fullfile(root, "native"));
end
