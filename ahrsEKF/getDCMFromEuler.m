%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% file:getDCMFromEuler.m
% date:2019/07/20
% author:YangYue
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function DCM = getDCMFromEuler(roll,pitch,yaw)
DCM = [cos(pitch)*cos(yaw),sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw),cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);
       cos(pitch)*sin(yaw),sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw),cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw);
       -sin(pitch),         sin(roll)*cos(pitch),                            cos(roll)*cos(pitch)];
end