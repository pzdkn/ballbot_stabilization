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

function setVMotorTorques(clientInfo, torques)
  
  torques(2) = -torques(2); % change direction due to defintion of model
  
  alpha = pi/4;     % work angle of wheel
  beta = pi/2;      % angle offset between axis of first wheel and x-axis of robot
  
  sa = sin(alpha);
  ca = cos(alpha);
  sb = sin(beta);
  cb = cos(beta);
  
  A = [  (2*cb)/(3*ca)             (2*sb)/(3*ca)             1/(3*sa);...
        -(cb+sqrt(3)*sb)/(3*ca)    (-sb+sqrt(3)*cb)/(3*ca)   1/(3*sa);...
         (-cb+sqrt(3)*sb)/(3*ca)  -(sb+sqrt(3)*cb)/(3*ca)    1/(3*sa)];     
  
  %torques = 1/3 * [ torques(3) + 2*(torques(1)*cb-torques(2)*sb)/ca;...
  %                  torques(3) + (sb*(-sqrt(3)*torques(1)+torques(2)) - cb*( torques(1)+sqrt(3)*torques(2)))/ca;...
  %                  torques(3) + (sb*( sqrt(3)*torques(1)+torques(2)) + cb*(-torques(1)+sqrt(3)*torques(2)))/ca];

  setMotorTorques(clientInfo, A*torques);

end