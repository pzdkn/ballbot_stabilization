function [nonlinear_func] = getNonlinearModel()
syms g l mAW mK rK rW ThetaKi ThetaWi AThetaAWx AThetaAWy AThetaAWz
param = [g,l,mAW,mK,rK,rW,ThetaKi,ThetaWi,AThetaAWx,AThetaAWy,AThetaAWz];
syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 u1 u2 u3

Nonlinear_Model_v1

m = getParameters("ETH");
params = [m.g,m.l,m.mAWs,m.mK,m.rK,m.rW,m.ThetaK,m.ThetaW,m.AThetaAWx,m.AThetaAWy,m.AThetaAWz];
F = subs(F, param, params);
nonlinear_func = matlabFunction(F,'File','nonlinear_fcn_v5_eth','Optimize', true);
end

% Vars', {[u1, u2, u3], [x1, x2, x3, x4, x5, x6, x7, x8, x9, x10]}