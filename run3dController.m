function [U, dt, max_t] = run3dController(q,r,rho)
%-----------------------------------------------------------------
% This function computes the gain matrix and evaluates the controller in
% vrep
%-----------------------------------------------------------------
% input:
%   q       [10x1] parameters for Q matrix
%   r       [3x1]  parameters for R matrix
%   rho     [1x1]  weight parameter between Q and R
%-----------------------------------------------------------------
% return:
%   U       [3xmax_t] Storage for input torques
%   dt      [1x1] time step
%   max_t   [1x1] Simulation time
%-----------------------------------------------------------------
% load system model
[A,B,C,D,m] = getModel();

% LQR Controller
Q = diag(q);
R = rho * diag(r);
sys = ss(A,B,C,D);
[K, ~, ~] = lqr(sys,Q,R);

% start simulation
clientInfo = startSimulation();

dt = 0.05; % time step in [s]
max_t = 10; % sim duration in [s]
U = []
iterations = 0;
for t=0:dt:max_t
  % get robot state
  [theta, dtheta, dtheta0, ball_pos, dphi,target] = getSimState3D(clientInfo);
  iterations = iterations +1;
%   if mod(iterations,10) == 0
%       disp("--------------------")
%       disp(["XYZ dtheta", dtheta0])
%       disp(["ZYX dtheta", dtheta])
%   end
  % get position of target in x y coordinates
  tpos = target(1:2);
  % get orientation of target as angle around z-axis
  tori = target(3);
  % target vector contains all target values for state x. The minus sign
  % for phi_x arises from the fact that negative phi_x yields positive y
  % displacement
  tz = theta(3);
  RLI =  [cos(tz), sin(tz), 0; -sin(tz), cos(tz), 0 ; 0, 0, 1]; 
  tphi = [-tpos(2)/m.rK;tpos(1)/m.rK;0];
  tphi = RLI*tphi;
  tvec = [0;0;0;0;tori;0;tphi(1);0;tphi(2);0];
  phi = [-ball_pos(2)/m.rK; ball_pos(1)/m.rK;0];
  phi = RLI*phi;
  x = [theta(1); dtheta(1); theta(2); dtheta(2); theta(3) ; dtheta(3);
      phi(1); dphi(1); phi(2); dphi(2)];
  e = tvec-x;
  % Controller
  %u = K*e;
  u = zeros(3,1);
  % setMotorTorques(clientInfo, u);
  % record data for plotting
  U = [U, u];
  % trigger simulation step
  clientInfo.vrep.simxSynchronousTrigger(clientInfo.clientID);  
end
endSimulation(clientInfo)
%% OLD 

% % Adapt to 2D K
% K_ = zeros(size(K))
% K_(:,1) = -K(:,3);
% K_(:,2) = -K(:,4);
% K_(:,3) = K(:,1);
% K_(:,4) = K(:,2);
% K_(:,5) = -K(:,5);
% K_(:,6) = -K(:,6);
% K_(:,7) = -K(:,9);
% K_(:,8) = -K(:,10);
% K_(:,9) = K(:,7);
% K_(:,10) = K(:,8);

% Adapt to ETH K
% K_ = -K;
% K_(:,5) = K(:,5);
% K_(:,6) = K(:,6);

