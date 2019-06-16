function [U, dt, max_t] = run2dController()
% load system model
m = getParameters("Alex_2D");

% Task a) compute system matrices
A = [0 1 0                                                           0;
     0 0 (-(m.c_2+m.c_4)*m.g*m.gamma)/(m.c_1*m.c_3-(m.c_2+m.c_4)^2)  0;
     0 0                   0                                         1;
     0 0 (m.c_1*m.g*m.gamma)/(m.c_1*m.c_3-(m.c_2+m.c_4)^2)           0];
       
b = [0;
     ( (m.c_2+m.c_3+m.c_4)*m.c_5)/(m.c_1*m.c_3-(m.c_2+m.c_4)^2) ;
     0;
     (-(m.c_1+m.c_2+m.c_4)*m.c_5)/(m.c_1*m.c_3-(m.c_2+m.c_4)^2) ];
       
C = eye(4);

d = zeros(4,1);

% Task b) compute LQR controller for xz- and yz-plane
Q = [ 20    0   0    0;...
       0  100   0    0;...
       0    0  10    0;...
       0    0   0   50];
       
R = 200;

Ky = lqr(A, b, Q, R)';
Kx = Ky;

% Task c) compute preamplifier gains
Vy = 1/(C(:,1)' * inv(b*Ky'-A) * b);
Vx = Vy;

% parameters for PD-controller in xy-planerot
kp = 0.5;
kd = 2.0*sqrt(m.c_m*kp);

% start simulation
clientInfo = startSimulation();

U = [];

dt = 0.05; % time step in [s]
max_t = 1; % sim duration in [s]
for t=0:dt:max_t
  % get robot state
   [theta, dtheta, ball_pos, dphi,target, Rz] = getSimState(clientInfo);
  
  % get position controller target [x y]
  target_pos = target(1:2);
  
  % get ball controller target theta_z
  target_or = target(3);

  % Task f) compute position controller
  e = target_pos-ball_pos; 
  phi = [ e(2)/m.rK;...
           -e(1)/m.rK;...
            0];
  phi = Rz*phi;
  
  % get system state
  yz =  [phi(1) dphi(1) theta(1) dtheta(1)]';
  xz = -[phi(2) dphi(2) theta(2) dtheta(2)]'; % change direction due to defintion of model
  xy =  [dphi(3) theta(3) dtheta(3)]';
  
  % Task d) compute new output
  u = [-Kx'*yz ;...
       -Ky'*xz ;...
       -(-kp*(target_or-xy(2)) - kd*(0-xy(3)))];

  u_real = setVMotorTorques(clientInfo, u);

  % record data for plotting
  U = [U u_real];

  % trigger simulation step
  clientInfo.vrep.simxSynchronousTrigger(clientInfo.clientID); 
end

endSimulation(clientInfo)

end

