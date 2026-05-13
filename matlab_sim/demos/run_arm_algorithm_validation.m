function results = run_arm_algorithm_validation(~, ~, doAnimate)
%RUN_ARM_ALGORITHM_VALIDATION 3pit-only arm algorithm validation entry point.
%
% The matlab_sim native path is intentionally specialized for the 3pit
% configuration. The old 6DOF Pieper validation path is retired.

if nargin < 3 || isempty(doAnimate)
    doAnimate = true;
end

results = run_3pit_control_validation(doAnimate);
end
