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

function [theta,dtheta ,ball_pos,ball_vel,target] = getSimState(clientInfo)
  
  [res retInts robot_state retStrings retBuffer] = simCallScriptFunction(clientInfo, 'get_sim_state',[],[],[],'');
  %Transformation to robot local frame is skipped, to see if it work
  %without it. Don't know if its valid to rotate angles.
  
  %theta
  alpha = robot_state(4);
  beta = robot_state(5);
  gamma = robot_state(6);
  [theta_x, theta_y, theta_z] = convertEulerAngle(alpha,beta,gamma);
  theta = [theta_x, theta_y, theta_z];
  %theta = [alpha, beta, gamma];
  %theta = [gamma, beta, alpha];
  %theta = [beta, gamma, alpha];
  %dtheta
  dalpha_dx = robot_state(10);
  dbeta_dy = robot_state(11);
  dgamma_dz = robot_state(12);
  [dtheta_x, dtheta_y, dtheta_z] = convertEulerAngle(dalpha_dx, dbeta_dy,dgamma_dz);
  dtheta = [dtheta_x, dtheta_y, dtheta_z];
  %dtheta = [dalpha_dx, dbeta_dy, dgamma_dz];
  %dtheta = [dgamma_dz, dbeta_dy, dalpha_dx];
  %dtheta = [dbeta_dy, dgamma_dz, dalpha_dx];
  ball_pos = robot_state(13:14);
  ball_vel = robot_state(7:9);
  target = robot_state(16:18);
  
  

end