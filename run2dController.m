function [U, t] = run2dController()
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

  u_real = setVMotorTorques(clientInfo, u);

  % record data for plotting
  U = [U u_real];

  % trigger simulation step
  simxSynchronousTrigger(clientInfo.clientID);  
end

endSimulation(clientInfo)

end

