%-----------------------------------------------------------------
% This sets directly the motor torques
%-----------------------------------------------------------------
% input:
%   clientInfo  [3x1] Handle for communication with simulator
%   torques     [1x3] Sets desired torques of each motor
%-----------------------------------------------------------------
% return:
%-----------------------------------------------------------------

function setMotorTorques(clientInfo, torques)

  simCallScriptFunction(clientInfo, 'set_motor_torques', [], torques, [],'');

end