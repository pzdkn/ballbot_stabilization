clc; clear; close all;

% load system model
[A,B,C,D] = getModel();


% LQR Controller
Q = diag([50 , 20, 50, 20, 100, 500, 100, 500, 100, 101]);
R = eye(3,3);

sys = ss(A,B,C,D);
[K,~,~] = lqr(sys,Q,R);

%{
%Task c) compute preamplifier gains
Vy = 1/(C(:,1)' * inv(b*Ky'-A) * b)
Vx = Vy
%}
%{
% parameters for PD-controller in xy-planerot
kp = 0.5;
kd = 2.0*sqrt(m.c_m*kp);
%}

% start simulation
clientInfo = startSimulation();

P = [];
YZ = [];
XZ = [];
XY = [];
DP = [];
DS = [];
U = [];
E = [];

dt = 0.05; % time step in [s]
max_t = 20.0; % sim duration in [s]
for t=0:dt:max_t
  % get robot state
  [s, Rz, ball_pos, target] = getSimState(clientInfo);
  
  % get position controller target [x y phi_z]
  d_p = [target(1:2); 0];
  
  % get ball controller target [phi_x phi_y phi_z]
  d_s = [0 0 target(3)]';

  % Task f) compute position controller
  e = d_p(1:2)-ball_pos(1:2); 
  d_phi = [ e(2)/m.rK;...
           -e(1)/m.rK;...
            0];
  d_phi = Rz*d_phi;
  
  % get system state
  yz =  [d_phi(1) s(7) s(4) s(10)]';
  xz = -[d_phi(2) s(8) s(5) s(11)]'; % change direction due to defintion of model
  xy =  [s(3) s(9) s(6) s(12)]';
  
  % Task d) compute new output
  u = [-Kx'*yz + Vx*d_s(1);...
       -Ky'*xz + Vy*d_s(2);...
       -(-kp*(d_s(3)-xy(3)) - kd*(0-xy(4)))];

  setVMotorTorques(clientInfo, u);

  % record data for plotting
  P = [P ball_pos];
  YZ = [YZ yz];
  XZ = [XZ xz];
  XY = [XY xy];
  DP = [DP d_p];
  DS = [DS d_s];
  U = [U u];
  E = [E e];

  % trigger simulation step
  clientInfo.vrep.simxSynchronousTrigger(clientInfo.clientID);  
end

endSimulation(clientInfo)

% plot results
plot_data
