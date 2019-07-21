%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% file:quaternionMulQuaternion.m
% date:2019/07/20
% author:YangYue
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function quat = quaternionMulQuaternion(quat1, quat2)

quat = zeros(4, 1);

p0 = quat1(1);
p1 = quat1(2);
p2 = quat1(3);
p3 = quat1(4);

q0 = quat2(1);
q1 = quat2(2);
q2 = quat2(3);
q3 = quat2(4);

quat(1) = p0*q0 - p1*q1 -p2*q2 - p3*q3;
quat(2) = p0*q1 + p1*q0 + p2*q3 - p3*q2;
quat(3) = p0*q2 + p2*q0 + p3*q1 - p1*q3;
quat(4) = p0*q3 + p3*q0 + p1*q2 - p2*q1;

end