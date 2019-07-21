%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% file:dataInit.m
% date:2019/07/20
% author:YangYue
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% load data
load data.mat
clearvars -except IMU IMU_label MAG MAG_label GPS GPS_label EKF1 EKF1_label

%% data length
[imuLength,~] = size(IMU);
[magLength,~] = size(MAG);
[gpsLength,~] = size(GPS);
[EKFLength, ~] = size(EKF1);
%% data index
imuIndex = 1;
magIndex = 1;
gpsIndex = 1;
ekfIndex= 1;
%% data synchronization
if MAG(1,2) < GPS(1,2)
    startTime = GPS(1,2);
    for i=1:magLength
        if MAG(i,2) <= startTime && MAG(i+1, 2) > startTime
            magIndex= i;
            break;
        end
    end
else
    startTime = MAG(1,2);
    for i=1:gpsLength
        if GPS(i,2) <= startTime && GPS(i+1, 2) > startTime
            gpsIndex = i;
            break;
        end
    end
end

%IMU start time
for i=1:imuLength
    if startTime <= IMU(i,2)
        imuIndex = i;
        break;
    end
end

if MAG(magIndex, 2) < IMU(imuIndex, 2)
    magIndex= magIndex+ 1;
end

if GPS(gpsIndex, 2) < IMU(imuIndex, 2)
    gpsIndex = gpsIndex + 1;
end
%% init quaternion
accel = IMU(imuIndex, 6:8);
initialRoll = atan2(-accel(2), -accel(3));
initialPitch = atan2(accel(1), -accel(3));

mag = MAG(magIndex, 3:5);
hx = mag(1)*cos(initialPitch) + mag(2)*sin(initialPitch)*sin(initialRoll) + mag(3)*sin(initialPitch)*cos(initialRoll);
hy = mag(2)*cos(initialRoll) - mag(3)*sin(initialRoll);
initialYaw = atan2(-hy, hx);
if(initialYaw < 0)
    initialYaw = initialYaw + 2*pi;
end
% from body to earth
DCM = getDCMFromEuler(initialRoll, initialPitch, initialYaw);
q0 = 0.5*sqrt(1+DCM(1,1)+DCM(2,2)+DCM(3,3));
q1 = 0.5*sqrt(1+DCM(1,1)-DCM(2,2)-DCM(3,3))*sign(DCM(3,2)-DCM(2,3));
q2 = 0.5*sqrt(1-DCM(1,1)+DCM(2,2)-DCM(3,3))*sign(DCM(1,3)-DCM(3,1));
q3 = 0.5*sqrt(1-DCM(1,1)-DCM(2,2)+DCM(3,3))*sign(DCM(2,1)-DCM(1,2));
% normalize
quat = normalizeQuaternion([q0,q1,q2,q3]);

%% readGpsData
   deg2rad = single(pi/180);
   earthRadius = single(6378145); % earth radius
   GndSpd = GPS(:,12);
   CourseDeg = GPS(:,13);
   VelD= GPS(:,14);
   for i= 2:length(GPS)
    LatDelta(i)   =  GPS(i,8) - GPS(1,8);
    LongDelta(i)  =  GPS(i,9) - GPS(1,9);
    posNED(1,i) = earthRadius * LatDelta(i)/100;% m
    posNED(2,i) = earthRadius * cos(GPS(1,8)*deg2rad) * LongDelta (i)/100;% m
    velNED(1,i) = GndSpd(i)*cos(CourseDeg(i)*deg2rad);% m/s
    velNED(2,i) = GndSpd(i)*sin(CourseDeg(i)*deg2rad);% m/s
    velNED(3,i) = VelD(i);% m/s
   end
   posNED(3,:) =  GPS(:,10);% m
   
 %% init velocity and position
 initVel = velNED(1:3,gpsIndex);
 initPos = posNED(1:3,gpsIndex);