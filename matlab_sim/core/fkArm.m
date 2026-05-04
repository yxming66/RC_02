function T = fkArm(modelSpec, q)
%FKARM Compute end-effector transform for a modelSpec.

modelSpec = loadRobotModel(modelSpec);
q = normalizeJointVector(q, modelSpec.dof);
T = getTransform(modelSpec.robot, q, modelSpec.eeName, modelSpec.baseName);
if isfield(modelSpec, "tcp") && ~isempty(modelSpec.tcp)
    T = T * modelSpec.tcp;
end
end

