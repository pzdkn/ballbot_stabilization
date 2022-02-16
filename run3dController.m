function [O] = run3dController(q,r,rho,max_t,dt)
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
[K, ~, e] = lqr(sys,Q,R);
% This is some hack to stabilize all that 
K(:,5:6) = -K(:,5:6);
% start simulation
clientInfo = startSimulation();

U = [];
X = [];
E = [];
for t=0:dt:max_t
  % get robot state
  [theta, dtheta, ball_pos, dphi,target] = getSimState3D(clientInfo);
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
  u = [0 0 1; 1 0 0; 0 1 0]*K*e;
  setMotorTorques(clientInfo, u);
  % record data for plotting
  U = [U, u];
  X = [X, x];
  E = [E, e];
  % trigger simulation step
  clientInfo.vrep.simxSynchronousTrigger(clientInfo.clientID);  
end
endSimulation(clientInfo)
O.X = X;
O.E = E;
O.U = U;
O.T = tvec;
end
