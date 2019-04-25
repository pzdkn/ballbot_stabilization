%-----------------------------------------------------------------
% This function reads the current robot state from simulation
%-----------------------------------------------------------------
% input:
%   clientInfo  [3x1] Handle for communication with simulator
%-----------------------------------------------------------------
% return:
%   x         [1x12]  Current state of BallRobot x = [theta_x; dtheta_x; theta_y; dtheta_y; theta_z
%                                                       dtheta_z;phi_x;dphi_x;phi_y;dphi_y]'
%                     phi: Orientation of ball
%                     theta: Orientation of body
%   ball_pos  [2x1]   Current ball position
%   target    [3x1]   Current target position
%-----------------------------------------------------------------

function [x, ball_pos, target] = getSimState(clientInfo)
  
  [res retInts robot_state retStrings retBuffer] = simCallScriptFunction(clientInfo, 'get_sim_state',[],[],[],'');

  x = [robot_state(4);robot_state(10);robot_state(5);robot_state(11);...
      robot_state(6);robot_state(12);robot_state(1);robot_state(7);...
      robot_state(2);robot_state(8)];
  
  ball_pos = robot_state(13:14)';

  target = robot_state(16:18)';

end