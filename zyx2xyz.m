function [alpha, beta, gamma] = zyx2xyz(theta_x, theta_y, theta_z)



end

%% OLD
% %-----------------------------------------------------------------
% % Convert from zyx euler angle sequence to xyz euler angle sequence
% % xyz is used by vrep, whereas zyx is used during modeling.
% % Conversion formulas adapted from : 
% % https://math.stackexchange.com/questions/3081609/given-tait-bryan-angles-x-y-z-intrinsic-how-can-i-get-tait-bryan-an
% %-----------------------------------------------------------------
% % input:
% %   theta_x, theta_y, theta_z := scalar angles which describe
% %   zyx rotation. Where Q' = Rz(theta_z)*Ry(theta_y)*Rx(theta_
% %-----------------------------------------------------------------
% % return: 
% % alpha, beta, gamma := scalar angles which describe xyz rotation.
% % Where Q = Rx(alpha)*Ry(beta)*Rz(gamma)
% %-----------------------------------------------------------------
% alpha = atan2(-cos(theta_x)*sin(theta_y)*sin(theta_z) + cos(theta_z)*sin(theta_x), ...
%     cos(theta_y)*cos(theta_x));
% beta = asin(cos(theta_x)*cos(theta_z)*sin(theta_y) +sin(theta_x)*sin(theta_z));
% gamma = atan2(-cos(theta_z)*sin(theta_y)*sin(theta_x) + cos(theta_x)*sin(theta_z), ...
%     cos(theta_y)*cos(theta_z));