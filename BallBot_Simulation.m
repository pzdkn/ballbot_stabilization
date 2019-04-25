clc; clear; close all;

% load system model
[A,B,C,D,m] = getModel();


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

POS = [];
X = [];
TARGET_POS = [];
TARGET_ORI = [];
POS_ERR = [];
THETA_Z_ERR = [];
U = [];

dt = 0.05; % time step in [s]
max_t = 20.0; % sim duration in [s]
for t=0:dt:max_t
  % get robot state
  [x, ball_pos, target] = getSimState(clientInfo);
  
  % get position of target in x y coordinates
  target_pos = target(1:2);
  
  % get orientation of target as angle around z-axis
  target_ori = target(3);

  % residual for current position and targeted position 
  %TODO!: I might just use phi_x and phi_y instead of ball_pos
  pos_err = target_pos(1:2)-ball_pos(1:2); 
  phi_x_err = pos_err(1)/m.rK;
  phi_y_err = pos_err(2)/m.rK;
  theta_z_err = target_ori - x(5);
  
  % get system state 
  % x = [theta_x; dtheta_x; theta_y; dtheta_y; theta_z,... 
  %         dtheta_z;phi_x;dphi_x;phi_y;dphi_y]'
  % use position & orientation residuals 
  x(5) = theta_z_err;
  x(7) = phi_x_err;
  x(9) = phi_y_err;
  
  % Task d) compute new output
  u = -K*x;

  setMotorTorques(clientInfo, u);

  % record data for plotting
  POS = [POS ball_pos];
  X = [X x];
  TARGET_POS = [TARGET_POS target_pos];
  TARGET_ORI = [TARGET_ORI target_ori];
  POS_ERR = [POS_ERR pos_err];
  THETA_Z_ERR = [THETA_Z_ERR  theta_z_err];
  U = [U u];

  % trigger simulation step
  clientInfo.vrep.simxSynchronousTrigger(clientInfo.clientID);  
end

endSimulation(clientInfo)

% plot results
plot_data
