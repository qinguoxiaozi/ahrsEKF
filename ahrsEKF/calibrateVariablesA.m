%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% file:calibrateVariablesA.m
% date:2019/07/21
% author:YangYue
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function navigation = calibrateVariablesA(X, dx)

quat = [X(1), X(2), X(3), X(4)]';
droll = dx(1);
dpitch = dx(2);
dyaw = dx(3);

dq = getRotationVector([droll, dpitch, dyaw], 1);
quat = quaternionMulQuaternion(dq, quat);
navigation(1:4, 1)= quat / norm(quat);

end