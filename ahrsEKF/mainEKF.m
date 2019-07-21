%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% file:mainEKF.m
% date:2019/07/20
% author:YangYue
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% clear space
clear;
clc;
close all;

%% data init
dataInit;

%% state definition
stateA = 6;% deltaRoll, deltaPitch, deltaYaw, bgx, bgy, bgz
estimationParameterA = zeros(4,1);
%% noise
QA = diag([1e-1, 1e-1, 1e-1, 0.01, 0.01, 0.01]*1e-4);% stateA process noise
Rmag = 5;% mag measurement noise
Raccel = diag([0.1,1]*5);% accle measurement noise

%% First order Markov constant of sensor
constGyro = 25;
betaGyro = diag([1/constGyro, 1/constGyro, 1/constGyro]);
constAccel = 15;
betaAccel = diag([1/constAccel, 1/constAccel, 1/constAccel]);

%% State quantity setting
dxA = zeros(stateA, 1);% stateA
PA = eye(stateA)*1e0;% stateA covariance 
estimationParameterA  = quat;% estimation values
%% Storage settings
save euler
euler = zeros(imuLength,3);
euler(1,:) = [initialRoll, initialPitch, initialYaw];
pos = zeros(imuLength, 3);
vel = zeros(imuLength, 3);
biasGyro = zeros(imuLength, 3);
biasAccel = zeros(imuLength, 3);

%% Corresponding bool judgment
useAccel = 1;
useMag = 1;
useGPS = 1;
accelUpdate = 0;
magUpdate = 0;
gpsUpdate = 0;

%% start 
timeIMU = IMU(imuIndex,2);
magIndex = magIndex + 1 ;
for i=imuIndex+1:imuLength
    lastTimeIMU = timeIMU;% last step IMU time
    timeIMU = IMU(i,2);% current step IMU time
    dtIMU = (timeIMU - lastTimeIMU)/10^6;% s,IMU update rate
     
    gyro = IMU(i,3:5)' - dxA(4:6,1);
   % eliminate gyro bias
    accel = IMU(i,6:8)';
   %% stateA 
   estimationParameterA = quaternionUpdate(estimationParameterA,gyro,dtIMU);
   Cbn = convertQuaternion2DCM(estimationParameterA );
   %% state EKFA
    % state and covariance prediction
    FA= JFA(Cbn, betaGyro);
    stateTransferMatrix = eye(stateA) + FA*dtIMU;
    dxA = statePredictA(dxA, stateTransferMatrix);
    PA = stateTransferMatrix * PA * stateTransferMatrix' + QA;
    PA = (PA + PA') / 2;
    % accel observer update
    Z = accel;
    [roll_r, pitch_r] = getRollAndPitchFromAccel(Z);
    [roll, pitch, yaw] = getEulerFromDCM(Cbn);
    dz = [roll_r - roll, pitch_r - pitch]';
    JHa = zeros(2,6);
    JHa(1,1) = cos(yaw);
    JHa(1,2) = sin(yaw);
    JHa(2,1) = -sin(yaw);
    JHa(2,2) = cos(yaw);
    k = PA*JHa'/(JHa*PA*JHa' + Raccel);
    dxA= dxA + k*(dz - JHa*dxA);
    PA= (eye(stateA) - k*JHa)*PA;
    PA = (PA+PA')/2;
  %  mag observer update
    if( magIndex < magLength && MAG(magIndex,2) > lastTimeIMU && MAG(magIndex,2) <=timeIMU)
        temp = MAG(magIndex,3:5);
        [roll, pitch, yaw] = getEulerFromDCM(Cbn);
        yaw_r  = getYawFromMag(temp, roll, pitch);
        dz = yaw_r - yaw;
        JHm = [0,0,1,0,0,0];
        k = PA*JHm' / (JHm*PA*JHm' + Rmag);
        dxA = dxA + k*(dz - JHm*dxA);
        PA= (eye(stateA) - k*JHm)*PA;
        PA= (PA + PA') / 2;
        magIndex = magIndex + 1;
        magUpdate = 1;
    end
%     get estimationParameterA 
    if magUpdate==1
        magUpdate = 0;
        estimationParameterA= calibrateVariablesA(estimationParameterA, dxA);
        dxA(1:3, 1) = zeros(3,1);
    end
%     get store estimationParameterA 
    DCM = convertQuaternion2DCM( estimationParameterA);
    [roll1, pitch1, yaw1] = getEulerFromDCM(Cbn);
     euler(i,:) = [roll1, pitch1, yaw1];
     biasGyro(i,:) = [dxA(4), dxA(5), dxA(6)];
end

%% plot
% roll
figure()
hold on
grid on
plot(IMU(:,2)/10^6,euler(:,1)*180/pi,'r');
plot(EKF1(:,2)/10^6,EKF1(:,3),'b');
legend('EKF','truth');
title('roll');
box on
% pitch
figure()
hold on
grid on
plot(IMU(:,2)/10^6,euler(:,2)*180/pi,'r');
plot(EKF1(:,2)/10^6,EKF1(:,4),'b');
legend('EKF','truth');
title('pitch');
box on

% yaw
figure()
hold on
grid on
plot(IMU(:,2)/10^6,euler(:,3)*180/pi,'r');
plot(EKF1(:,2)/10^6,EKF1(:,5),'b');
legend('EKF','truth');
title('yaw');
box on
% gyro bias
figure()
hold on
grid on
plot(IMU(:,2)/10^6, biasGyro(:,1),'r');
plot(IMU(:,2)/10^6, biasGyro(:,2),'b');
plot(IMU(:,2)/10^6, biasGyro(:,3),'k');
title('gyro bias');
box on
legend('x', 'y', 'z');