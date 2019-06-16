function [U, t] = run3dController(q,r,rho)
% load system model
[A,B,C,D,m] = getModel();

% LQR Controller
Q = diag(q);
R = rho * eye(r);
sys = ss(A,B,C,D);
[K, ~, ~] = lqr(sys,Q,R);

% start simulation
clientInfo = startSimulation();

dt = 0.05; % time step in [s]
max_t = 1.0; % sim duration in [s]
x = zeros(10,1);
U = []
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
  % In the ETH thesis, dphi is the angular velocity whereas phi is the
  % integral thereof
  x(7) = ball_pos(2)/m.rK; %phi_x = p_y/rK where p_y := y position of ball (no slip condition)
  x(8) = ball_vel(1); %dphi_dx
  x(9) = ball_pos(1)/m.rK; %phi_y = p_x/rK where p_x := x position of ball (no slip condition)
  x(10) = ball_vel(2); %dphi_dy
  % Controller
  u = -K*x;

  setMotorTorques(clientInfo, u);

  % record data for plotting
  U = [U, u];
  % trigger simulation step
  clientInfo.vrep.simxSynchronousTrigger(clientInfo.clientID);  
end
endSimulation(clientInfo)
