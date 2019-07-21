%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% file:quaternionUpdate.m
% date:2019/07/21
% author:YangYue
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function X_new = quaternionUpdate(X, gyro,dt)

quat = X(1:4,1);

dq = getRotationVector(gyro, dt);

quat_new = quaternionMulQuaternion(quat, dq);

X_new(1:4, 1) = quat_new / norm(quat_new);

end