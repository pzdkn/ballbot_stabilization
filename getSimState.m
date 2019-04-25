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
  %Transformation to robot local frame is skipped, to see if it work
  %without it. Don't know if its valid to rotate angles.
  theta_x = robot_state(4);
  dtheta_x = robot_state(10);
  theta_y = robot_state(5);
  dtheta_y = robot_state(11);
  theta_z = robot_state(6);
  dtheta_z = robot_state(12);
  phi_x = robot_state(1);
  dphi_x = robot_state(7);
  phi_y = robot_state(2);
  dphi_y = robot_state(8);
 
  x=[theta_x; dtheta_x; theta_y; dtheta_y; theta_z;...
      dtheta_z;phi_x;dphi_x;phi_y;dphi_y];
  
  ball_pos = robot_state(13:14)';

  target = robot_state(16:18)';
  
  %instead of returning x as measured, we return directly the residuals
  %from targeted position and orientation 
  

end