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

function [theta, dtheta, ball_pos, dphi,target, Rz] = getSimState(clientInfo)
  
  [res retInts robot_state retStrings retBuffer] = simCallScriptFunction(clientInfo, 'get_sim_state',[],[],[],'');
  
  %theta := body orientation
  alpha = robot_state(4);
  beta = robot_state(5);
  gamma = robot_state(6);
  [theta_x, theta_y, theta_z] = convertEulerAngle(alpha,beta,gamma);
  Rz = [ cos(theta_z)  -sin(theta_z) 0;...
         sin(theta_z)   cos(theta_z) 0;...
         0            0          1]';
  theta = Rz*[theta_x; theta_y; theta_z];
  %dtheta := body angular velocity
  dalpha_dx = robot_state(10);
  dbeta_dy = robot_state(11);
  dgamma_dz = robot_state(12);
  [dtheta_x, dtheta_y, dtheta_z] = convertEulerAngle(dalpha_dx, dbeta_dy,dgamma_dz);
  dtheta = Rz*[dtheta_x; dtheta_y; dtheta_z];
  
  % Here we convert measured angular velocity of the body frame to
  % derivatives of the state variables
%   J_inv = [1, sin(theta_x)*tan(theta_y), cos(theta_x)*tan(theta_y);
%             0, cos(theta_x) , -sin(theta_x);
%             0, sin(theta_x)/cos(theta_y), cos(theta_x)/cos(theta_y)];
%   dtheta = J_inv * dtheta;

  % ball position 
  ball_pos = robot_state(13:14);
  % dphi := ball angular velocity
  dphi = Rz*robot_state(7:9)';
  % target position
  target = robot_state(16:18);
  
end