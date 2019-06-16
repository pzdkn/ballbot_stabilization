function [U, dt, max_t] = run3dController(q,r,rho)
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
max_t = 1; % sim duration in [s]
x = zeros(10,1);
U = []
for t=0:dt:max_t
  % get robot state
  [theta, dtheta, ball_pos, dphi,target, Rz] = getSimState(clientInfo);
  
  % get position of target in x y coordinates
  target_pos = target(1:2);
  
  % get orientation of target as angle around z-axis
  target_ori = target(3);
  
  e = target_pos - ball_pos
  phi = [ e(2)/m.rK;...
           -e(1)/m.rK;...
            0];
  phi = Rz*phi
  x = [theta(1); dtheta(1); theta(2); dtheta(2); theta(3); dtheta(3);
      phi(1); dphi(1); phi(2); dphi(2)];
  
  % Controller
  u = -K*x;
  setMotorTorques(clientInfo, u);
  % record data for plotting
  U = [U, u];
  % trigger simulation step
  clientInfo.vrep.simxSynchronousTrigger(clientInfo.clientID);  
end
endSimulation(clientInfo)
