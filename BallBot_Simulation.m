clc; clear; close all;

% load system model
[A,B,C,D,m] = getModel();

% LQR Controller
Q = 0.5*diag([1000 , 500, 1000, 500, 40, 20, 20, 10, 20, 10]);
R = 100*eye(3,3);

sys = ss(A,B,C,D);
[K, ~, ~] = lqr(sys,Q,R);

% K = [22.34, 4.63, 0.0, 0.0, -0.36, -0.29, 0.36, 0.53, 0.0, 0.0;
%     -11.17, -2.32, 19.34, 4.01, -0.36, -0.29, -0.18, -0.26, 0.31, 0.46;
%     -11.17, -2.32, -19.34, -4.01, -0.36, -0.29, -0.18, -0.26, -0.31, -0.46];


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
  x(1) = theta(1); %theta_x
  x(2) = dtheta(1); %dtheta_dx
  x(3) = theta(2); %theta_y
  x(4) = dtheta(2); %dtheta_dy
  x(5) = theta(3); %theta_z
  x(6) = dtheta(3); %dtheta_dz
  %In the ETH thesis, dphi is the angular velocity whereas phi is the
  %integral thereof
  x(7) = ball_pos(2)/m.rK; %phi_x = p_y/rK where p_y := y position of ball (no slip condition)
  %x(7) = 0;
  x(8) = ball_vel(1); %dphi_dx
  %x(8) = 0;
  x(9) = ball_pos(1)/m.rK; %phi_y = p_x/rK where p_x := x position of ball (no slip condition)
  %x(9) = 0
  x(10) = ball_vel(2); %dphi_dy
  %x(10) = 0;

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
