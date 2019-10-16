%-----------------------------------------------------------------
% This transforms torque commands of the virtual motors to the real robot and
% afterwards applies it.
% Virtual motor 1: Rotates around x-axis (driving forwards)
% Virtual motor 2: Rotates around y-axis (driving sidwards)
% Virtual motor 3: Rotates around z-axis (turning in place)
%-----------------------------------------------------------------
% input:
%   clientInfo  [3x1] Handle for communication with simulator
%   torques     [1x3] Sets desired torques of each virtual motor
%-----------------------------------------------------------------
% return:
%-----------------------------------------------------------------

function [real_torques]=setVMotorTorques(clientInfo, torques)
  
  %torques(2) = -torques(2); % change direction due to defintion of model
  alpha = pi/4;     % work angle of wheel
  beta = pi/2;      % angle offset between axis of first wheel and x-axis of robot
  
  sa = sin(alpha);
  ca = cos(alpha);
  sb = sin(beta);
  cb = cos(beta);
  
  T = [  (2*cb)/(3*ca)             (2*sb)/(3*ca)             1/(3*sa);...
        -(cb+sqrt(3)*sb)/(3*ca)    (-sb+sqrt(3)*cb)/(3*ca)   1/(3*sa);...
         (-cb+sqrt(3)*sb)/(3*ca)  -(sb+sqrt(3)*cb)/(3*ca)    1/(3*sa)];     
  real_torques = T*torques;
  setMotorTorques(clientInfo, real_torques);
end