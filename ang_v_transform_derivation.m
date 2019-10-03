%% Derivation of Transformation for Conversion to Euler angle derivative
% Asssuming we obtain the angular velocity wrt absolute frame with
% simGetVelocity in VREP, then we need to convert it back to Euler angle
% derivatives. This can be done using T. The formulas can be derived from
% the p.30 of Ballbot thesis.
syms tx ty tz
J = [1, 0, -sin(ty); 0, cos(tx), cos(ty)*sin(tx); 0, -sin(tx), cos(tx)*cos(ty)];
Jinv = simplify(inv(J));
Rx = [1, 0, 0; 0 cos(tx) -sin(tx);0 sin(tx) cos(tx)];
Ry = [cos(ty) 0 sin(ty); 0 1 0;-sin(ty) 0 cos(ty)];
Rz = [cos(tz) -sin(tz) 0; sin(tz) cos(tz) 0; 0 0 1];
RIA = Rz*Ry*Rx;
RAI = transpose(RIA);
T = simplify(Jinv*RAI)
%% Derivation of Euler XYZ derivative to Euler ZYX derivative
% Assuming we obtain a XYZ Euler derivative. We obtain ZYX Euler derivative
% by first converting it to angular velocity wrt world frame, then
% convert that to ZYX derivative via inverse Jacobian. We have to be
% careful with singularities here.
% We have : J_xyz*dtheta_xyz = omega_IA = J_zyx * dtheta_zyx
% -> dtheta_zyx = inv(J_zyx)*J_xyz*dtheta_xyz
syms tx ty tz
J_xyz = [1 0 sin(ty); 0 cos(tx) -cos(ty)*sin(tx); 0 sin(tx) cos(tx)*cos(ty)];
J_zyx = [0 -sin(tz) cos(ty)*cos(tz); 0 cos(tz) cos(ty)*sin(tz); 1 0 -sin(ty)];
T = simplify(inv(J_zyx)*J_xyz);