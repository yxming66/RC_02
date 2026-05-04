function out = animateArmTrajectory(modelSpec, qSamples, titleText, pauseSeconds)
%ANIMATEARMTRAJECTORY Animate a joint trajectory and draw the TCP path.

if nargin < 3 || isempty(titleText)
    titleText = "arm trajectory";
end
if nargin < 4 || isempty(pauseSeconds)
    pauseSeconds = 0.02;
end

modelSpec = loadRobotModel(modelSpec);
qSamples = double(qSamples);
if size(qSamples, 2) ~= modelSpec.dof
    error("animateArmTrajectory:InvalidSamples", ...
        "qSamples must be N-by-dof.");
end

n = size(qSamples, 1);
tcp = zeros(n, 3);
for i = 1:n
    T = fkArm(modelSpec, qSamples(i, :));
    tcp(i, :) = tform2trvec(T);
end

fig = figure("Name", titleText);
ax = axes("Parent", fig);
for i = 1:n
    show(modelSpec.robot, qSamples(i, :), ...
        "Parent", ax, "PreservePlot", false, "Frames", "off");
    hold(ax, "on");
    plot3(ax, tcp(1:i, 1), tcp(1:i, 2), tcp(1:i, 3), ...
        "r-", "LineWidth", 1.5);
    plot3(ax, tcp(1, 1), tcp(1, 2), tcp(1, 3), ...
        "go", "MarkerFaceColor", "g");
    plot3(ax, tcp(end, 1), tcp(end, 2), tcp(end, 3), ...
        "bx", "MarkerSize", 9, "LineWidth", 1.5);
    hold(ax, "off");
    axis(ax, "equal");
    grid(ax, "on");
    view(ax, 135, 25);
    title(ax, sprintf("%s  (%d/%d)", titleText, i, n), "Interpreter", "none");
    drawnow;
    if pauseSeconds > 0
        pause(pauseSeconds);
    end
end

out = struct();
out.figure = fig;
out.axes = ax;
out.tcpPath = tcp;
end

