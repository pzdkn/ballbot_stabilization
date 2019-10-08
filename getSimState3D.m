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
function [theta, dtheta, ball_pos, dphi,target] = getSimState(clientInfo)
  [res retInts robot_state retStrings retBuffer] = simCallScriptFunction(clientInfo, 'get_sim_state',[],[],[],'');
  x = robot_state(4); y= robot_state(5); z = robot_state(6); 
  w = robot_state(7); 
  q = [x,y,z,w];
  theta = quat2eul(q, 'rzyx');
  theta_ = quat2eul(q, 'rxyz');
  dphi = [robot_state(8), robot_state(9), robot_state(10)];
  dtheta = [robot_state(11), robot_state(12), robot_state(13)];
  ball_pos = [robot_state(14), robot_state(15)];
  target = [robot_state(17), robot_state(18), robot_state(19)];
end