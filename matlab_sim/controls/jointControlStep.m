function out = jointControlStep(modelSpec, q)
%JOINTCONTROLSTEP Evaluate a joint-space command.

q = normalizeJointVector(q, modelSpec.dof);
T = fkArm(modelSpec, q);

out = struct();
out.q = q;
out.T = T;
out.pose = tformToPose(T);
out.success = jointWithinLimits(modelSpec, q);
end

