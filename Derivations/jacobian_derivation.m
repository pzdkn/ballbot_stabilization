%% Derivation of the Jacobian for the ZYX intrinsic system
% We need the inverse of the ZYX Jacobian to map angular velocity to ZYX
% Euler angle derivatives
syms tx ty tz
J_zyx = [0 -sin(tz) cos(ty)*cos(tz); 0 cos(tz) cos(ty)*sin(tz); 1 0 -sin(ty)];
Jinv_zxy = simplify(inv(J_zyx));