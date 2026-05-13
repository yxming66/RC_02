function pose = tformToPose(T)
%TFORMTOPOSE Convert homogeneous transform to [x y z roll pitch yaw].

translation = tform2trvec(T);
zyx = tform2eul(T, "ZYX");
pose = [translation, zyx(3), zyx(2), zyx(1)];
end

