function T = poseToTform(pose)
%POSETOTFORM Convert [x y z roll pitch yaw] to homogeneous transform.

pose = double(pose(:).');
if numel(pose) ~= 6
    error("poseToTform:InvalidPose", "Pose must be [x y z roll pitch yaw].");
end

translation = pose(1:3);
rollPitchYaw = pose(4:6);
T = trvec2tform(translation) * eul2tform([rollPitchYaw(3), rollPitchYaw(2), rollPitchYaw(1)], "ZYX");
end

