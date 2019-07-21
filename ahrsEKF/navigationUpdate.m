%% 进行导航参数的更新
% 使用定时采样增量的毕卡求解法
function X_new = navigationUpdate(X, gyro,accel,  dt)

%% for ahrs
quat = X(1:4,1);
%bigTheta = getBigTheta(gyro, dt);
%quat_new = bigTheta*quat;
dq = getRotationVector(gyro, dt);

quat_new = quaternionMulQuaternion(quat, dq);
% 注意要归一化
X_new(1:4, 1) = quat_new / norm(quat_new);

%% for pos and vel
quat_new = quat_new / norm(quat_new);
Cbn = convertQuaternion2DCM(quat_new);
accel_ef = Cbn * accel + [0, 0, 9.80665]';
deltaVel = accel_ef * dt;
vel = X(5:7, 1) + deltaVel;
X_new(5:7, 1) = vel;

% update pos
pos = X(8:10, 1) + 0.5*(vel + X(5:7, 1))*dt;
X_new(8:10, 1) = pos;

end