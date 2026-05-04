function poses = makeCartesianLineTrajectory(startPose, goalPose, steps)
%MAKECARTESIANLINETRAJECTORY Linear pose interpolation in xyz/rpy.

if nargin < 3
    steps = 50;
end

startPose = double(startPose(:).');
goalPose = double(goalPose(:).');
s = linspace(0, 1, steps).';
poses = startPose + s .* (goalPose - startPose);
end

