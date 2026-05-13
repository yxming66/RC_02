function traj = makeJointTrajectory(q0, q1, steps)
%MAKEJOINTTRAJECTORY Linear joint trajectory samples.

if nargin < 3
    steps = 50;
end

q0 = double(q0(:).');
q1 = double(q1(:).');
s = linspace(0, 1, steps).';
traj = q0 + s .* (q1 - q0);
end

