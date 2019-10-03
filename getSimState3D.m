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
function [theta, dtheta,dtheta0, ball_pos, dphi,target] = getSimState(clientInfo)
  [res retInts robot_state retStrings retBuffer] = simCallScriptFunction(clientInfo, 'get_sim_state',[],[],[],'');
  x = robot_state(4); y= robot_state(5); z = robot_state(6); 
  w = robot_state(7); 
  q = [x,y,z,w];
  theta = quat2eul(q, 'rzyx');
  dphi = [robot_state(8), robot_state(9), robot_state(10)];
  dtheta0 = [robot_state(11), robot_state(12), robot_state(13)];
  dq = eul2quat(robot_state(11), robot_state(12), robot_state(13), 'rxyz');
  dtheta = quat2eul(dq, 'rzyx');
  
  ball_pos = [robot_state(14), robot_state(15)];
  target = [robot_state(17), robot_state(18), robot_state(19)];
end
%% Code Dump
% Computing angular velocity of ball from Jacobian
%   %angular rate of the ball
%   [~ ,~, br, ~, ~] = simCallScriptFunction(clientInfo, 'get_ball_rate',[],[],[],'');
%   [~ ,~, bor, ~, ~] = simCallScriptFunction(clientInfo, 'get_ball_orientation',[],[],[],'');
%   % Jacobian to map to angular velocity
%   J = [1, 0, sin(bor(2));
%       0, cos(bor(1)), -cos(bor(2))*sin(bor(1)); ...
%        0, sin(bor(1)), cos(bor(1))*cos(bor(2))];
%   dphi0 = J*br(:);
%   dphi0 = dphi0';
%   disp("--------------------")
%       disp(["Angle Axis", dphi])
%       disp(["Jacobian", dphi0])
%       disp(["Euler Angle Rate", br])
 % -----------------------------------------------------