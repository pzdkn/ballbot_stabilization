clc; clear; close all;

% load system model
[A,B,C,D,m] = getModel();

% LQR Controller
Q = diag([1000 , 500, 1000, 500, 40, 20, 20, 10, 20, 10]);
R = 100*eye(3,3);

sys = ss(A,B,C,D);
[K, ~, ~] = lqr(sys,Q,R);




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
%placeholder for state variables
x = zeros(10,1);
for t=0:dt:max_t
  % get robot state
  [theta, dtheta, ball_pos, ball_vel, target] = getSimState(clientInfo);
  
  % get position of target in x y coordinates
  target_pos = target(1:2);
  
  % get orientation of target as angle around z-axis
  target_ori = target(3);
  
  %set up state vector
  x(1) = theta(1);
  x(2) = dtheta(1);
  x(3) = theta(2);
  x(4) = dtheta(2);
  x(5) = theta(3);
  x(6) = dtheta(3);
  x(7) = ball_pos(1)/m.rK;
  x(8) = ball_vel(1)/m.rK;
  x(9) = ball_pos(2)/m.rK;
  x(10) = ball_vel(2)/m.rK;

  u = -K*x;

  setMotorTorques(clientInfo, u);

  % record data for plotting
  POS = [POS ball_pos];
  X = [X x];
  TARGET_POS = [TARGET_POS target_pos];
  TARGET_ORI = [TARGET_ORI target_ori];
  %POS_ERR = [POS_ERR pos_err];
  %THETA_Z_ERR = [THETA_Z_ERR  theta_z_err];
  U = [U u];

  % trigger simulation step
  clientInfo.vrep.simxSynchronousTrigger(clientInfo.clientID);  
end

endSimulation(clientInfo)

% plot results
plot_data
