%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% file:getRotationVector.m
% date:2019/07/21
% author:YangYue
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function rotationVector = getRotationVector(w, dt)

rotationVector = zeros(4, 1);
theta = w*dt;
thetax = theta(1);
thetay = theta(2);
thetaz = theta(3);
theta_norm = sqrt(thetax*thetax + thetay*thetay + thetaz*thetaz);

if theta_norm > 0
    rotationVector(1) = cos(0.5*theta_norm);
    rotationVector(2) = thetax*sin(0.5*theta_norm) / theta_norm;
    rotationVector(3) = thetay*sin(0.5*theta_norm) / theta_norm;
    rotationVector(4)  = thetaz*sin(0.5*theta_norm) / theta_norm;
else
    rotationVector(1) = 1;
    rotationVector(2) = 0;
    rotationVector(3) = 0;
    rotationVector(4)  = 0;
end

end