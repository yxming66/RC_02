function out = cartesianControlStep(modelSpec, targetPose, seed, solverSpec)
%CARTESIANCONTROLSTEP Solve IK for a Cartesian target pose.

targetTform = poseToTform(targetPose);
[q, info] = solveIK(modelSpec, targetTform, seed, solverSpec);
check = checkIkSolution(modelSpec, q, targetTform, solverSpec);

out = struct();
out.q = q;
out.info = info;
out.check = check;
out.targetTform = targetTform;
out.actualTform = check.actualTform;
out.success = check.success;
end

