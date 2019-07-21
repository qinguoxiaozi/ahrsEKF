%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% file:getRollAndPitchFromAccel.m
% date:2019/07/21
% author:YangYue
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [roll, pitch] = getRollAndPitchFromAccel(accel)

ax = accel(1);
ay = accel(2);
az = accel(3);
roll = atan2(-ay, -az);
pitch = atan2(ax, -az);

end