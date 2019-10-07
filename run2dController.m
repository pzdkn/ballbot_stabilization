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
%% Transformation to convert to real torque
alpha = pi/4;     % work angle of wheel
beta = 2/3*pi;      % angle offset between axis of first wheel and x-axis of robot

sa = sin(alpha);
ca = cos(alpha);
sb = sin(beta);
cb = cos(beta);

T = [  (2*cb)/(3*ca)             (2*sb)/(3*ca)             1/(3*sa);...
        -(cb+sqrt(3)*sb)/(3*ca)    (-sb+sqrt(3)*cb)/(3*ca)   1/(3*sa);...
         (-cb+sqrt(3)*sb)/(3*ca)  -(sb+sqrt(3)*cb)/(3*ca)    1/(3*sa)];  
%%     
% start simulation
clientInfo = startSimulation();

U = [];

dt = 0.05; % time step in [s]
max_t = 20; % sim duration in [s]
for t=0:dt:max_t
  % get robot state
    [x, Rz, bpos, target] = getSimState2D(clientInfo);
    % get position controller target [x y]
    tphi = Rz*[-target(2)/m.rK; target(1)/m.rK;0];
    % get ball controller target theta_z
    tor = target(3);
    phi = Rz*[-bpos(2)/m.rK; bpos(1)/m.rK; 0];
    x = [x(4);x(10);x(5);x(11);x(6);x(12);phi(1);x(7);phi(2);x(8)];
    tar = [0;0;0;0;tor;0;tphi(1);0;tphi(2);0];
    % Test K Matrix :
    % K adapted reordered state variables based on 3d model
    K1 = [Kx(3), Kx(4), 0, 0, 0, 0, Kx(1), Kx(2), 0, 0];
    K2 = [0, 0, Ky(3), Ky(4), 0, 0, 0, 0, Ky(1), Ky(2)];
    K3 = [0, 0, 0, 0, kp, kd, 0, 0, 0, 0];
    K = [K1; K2; K3];
    %K = (T*diag([1,-1,1])*K);
    u = K*(tar-x);
    u = setVMotorTorques(clientInfo, u);
    U = [U u];
  % trigger simulation step
  clientInfo.vrep.simxSynchronousTrigger(clientInfo.clientID); 
end

endSimulation(clientInfo)

end

