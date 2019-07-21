%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% file:JFA.m
% date:2019/07/21
% author:YangYue
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function F = JFA(Cbn, betaG)

F = zeros(6);
F (1:3, 4:6) = -Cbn;
F(4:6, 4:6) = -betaG;

end