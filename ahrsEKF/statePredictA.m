%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% file:statePredictA.m
% date:2019/07/21
% author:YangYue
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function X_priori = statePredictA(X, stateTransferMatrix)

X_priori = stateTransferMatrix * X;

end