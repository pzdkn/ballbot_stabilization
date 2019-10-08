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
  K = [32.7967237481395,6.81869003123556,-4.80417049864658e-13,-1.10487392114583e-13,-0.129099444873580,-0.179025478053225,0.258198889747183,0.733803465029754,-8.27414403312820e-15,-2.33136774607709e-14;-16.3983618740701,-3.40934501561787,28.3974630078044,5.90219285866928,-0.129099444873581,-0.179025478053225,-0.129099444873598,-0.366901732514894,0.223606797749984,0.635462966777390;-16.3983618740694,-3.40934501561770,-28.3974630078040,-5.90219285866918,-0.129099444873581,-0.179025478053226,-0.129099444873588,-0.366901732514861,-0.223606797749976,-0.635462966777368];
  u = K*e;
  setMotorTorques(clientInfo, u);
  % record data for plotting
  U = [U, u];
  % trigger simulation step
  clientInfo.vrep.simxSynchronousTrigger(clientInfo.clientID);  
end
endSimulation(clientInfo)
