%-----------------------------------------------------------------
% This function reads the current robot state from simulation
%-----------------------------------------------------------------
% input:
%   clientInfo  [3x1] Handle for communication with simulator
%-----------------------------------------------------------------
% return:
%   x         [1x12]  Current state of BallRobot x = [ phi_x,  phi_y,  phi_z,  theta_x,  theta_y,  theta_z,
%                                                    dphi_x, dphi_y, dphi_z, dtheta_x, dtheta_y, dtheta_z]'
%                     phi: Orientation of ball
%                     theta: Orientation of body
%   Rz        [3x3]   Rotation from world to robot frame around z-Axis
%   ball_pos  [3x1]   Current ball position
%   target    [3x1]   Current target position
%-----------------------------------------------------------------

function [x, Rz, ball_pos, target] = getSimState2D(clientInfo)
  
  [res retInts robot_state retStrings retBuffer] = simCallScriptFunction(clientInfo, 'get_sim_state',[],[],[],'');

  % rotate all states given in world around z in body frame to get robot local system
  robot_state = robot_state';
  gamma = robot_state(6);
  Rz = [ cos(gamma)  -sin(gamma) 0;...
         sin(gamma)   cos(gamma) 0;...
         0            0          1]';
  %    ball orientation     body orientation     ball velocity        body velocity
  x = [Rz*robot_state(1:3); Rz*robot_state(4:6); Rz*robot_state(7:9); Rz*robot_state(10:12)];
  
  ball_pos = robot_state(13:15);

  target = robot_state(16:18);

end