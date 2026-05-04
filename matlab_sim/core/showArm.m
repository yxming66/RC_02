function ax = showArm(modelSpec, q, ax)
%SHOWARM Display the robot at q.

modelSpec = loadRobotModel(modelSpec);
q = normalizeJointVector(q, modelSpec.dof);

if nargin < 3 || isempty(ax)
    figure("Name", modelSpec.name);
    ax = axes();
end

show(modelSpec.robot, q, "Parent", ax, "PreservePlot", false, "Frames", "on");
axis(ax, "equal");
grid(ax, "on");
view(ax, 135, 25);
drawnow;
end

