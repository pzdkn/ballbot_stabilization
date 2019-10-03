function [theta_x, theta_y, theta_z] = xyz2zyx(alpha, beta, gamma)
%-----------------------------------------------------------------
% Convert from xyz euler angle sequence to zyx euler angle sequence
% xyz is used by vrep, whereas zyx is used during modeling.
% Conversion formulas adapted from : 
% https://math.stackexchange.com/questions/3081609/given-tait-bryan-angles-x-y-z-intrinsic-how-can-i-get-tait-bryan-an
%-----------------------------------------------------------------
% input:
%   alpha, beta, gamma := scalar angles which describe xyz rotation.
%   Where Q = Rx(alpha)*Ry(beta)*Rz(gamma)
%-----------------------------------------------------------------
% return: theta_x, theta_y, theta_z := scalar angles which describe
%   zyx rotation. Where Q' = Rz(theta_z)*Ry(theta_y)*Rx(theta_x)
%-----------------------------------------------------------------
theta_x =  atan2(cos(alpha)*sin(beta)*sin(gamma) ...
            + sin(alpha)*cos(gamma), cos(alpha)*cos(beta));
        
theta_y = asin(cos(alpha)*sin(beta)*cos(gamma) - sin(alpha)*sin(gamma));

theta_z = atan2(sin(alpha)*sin(beta)*cos(gamma) + cos(alpha)*sin(gamma),...
    cos(beta)*cos(gamma));
end

















