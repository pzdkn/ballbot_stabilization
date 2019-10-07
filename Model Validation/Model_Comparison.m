%% Model Comparison
% We compare the state evolution and the control output of the 3D and the
% 2D model, if we initialize with same initial conditions. We assume that
% if we initial conditions only concern with on plane, the 3D model will be
% unaffected by dynamics of the other planes. This assumption is supported
% by observations from PlotToVrep, where 3D Model will only swing within
% one plane, if it is only tilted within one plane.

%% Intialization
tx = pi/4; ty = 0; tz = 0; dtx = 0; dty = 0; dtz = 0;
px = 0; py = 0; dpx = 0; dpy = 0;
iterations = 10 ;
%% 2D Model
m = getParameters("Alex_2D");
A2D = [0 1 0                                                           0;
     0 0 (-(m.c_2+m.c_4)*m.g*m.gamma)/(m.c_1*m.c_3-(m.c_2+m.c_4)^2)  0;
     0 0                   0                                         1;
     0 0 (m.c_1*m.g*m.gamma)/(m.c_1*m.c_3-(m.c_2+m.c_4)^2)           0];
       
B2D = [0;
     ( (m.c_2+m.c_3+m.c_4)*m.c_5)/(m.c_1*m.c_3-(m.c_2+m.c_4)^2) ;
     0;
     (-(m.c_1+m.c_2+m.c_4)*m.c_5)/(m.c_1*m.c_3-(m.c_2+m.c_4)^2) ];
       
C2D = eye(4);

D2D = zeros(4,1);
%% 3D Model
[A3D,B3D,C3D,D3D,m] = getModel();
%% Compare State Evolution
tspan = [0,300];
% 2D ODE
ode2D = @(t,x2D) A2D*x2D;
x2D0 = [px; dpx; tx; dtx];
[t2D, x2D] = ode45(ode2D, tspan, x2D0);
% 3D ODE
ode3D = @(t,x3D) A3D*x3D;
x3D0 = [tx; dtx; ty; dty; tz; dtz; px; dpx; py; dpy];
[t3D, x3D] = ode45(ode3D, tspan, x3D0);
plot(t2D, x2D(:,1),t3D,x3D(:,7));
title("Phi")
figure()
plot(t2D, x2D(:,2),t3D,x3D(:,8));
title("DPhi")
figure()
plot(t2D, x2D(:,3),t3D,x3D(:,1));
title("Theta X")
figure()
plot(t2D, x2D(:,4),t3D,x3D(:,2));
title("DTheta X")
%% Compare Control Output
% First just compare the K gain matrices
% lqr parameters following variable order of 2D 
q = [20, 100, 10, 50];
r = 200;
% 2D Gain for real torques
Q2D = diag(q);
R2D = r;
Ky = lqr(A2D, B2D, Q2D, R2D)';
Kx = Ky;
K1 = [Kx(3), Kx(4), 0, 0, 0, 0, Kx(1), Kx(2), 0, 0];
K2 = [0, 0, Ky(3), Ky(4), 0, 0, 0, 0, Ky(1), Ky(2)];
K2D = [K1; K2; zeros(1,10)];
alpha = pi/4;     
beta = pi/3;     
sa = sin(alpha);
ca = cos(alpha);
sb = sin(beta);
cb = cos(beta);
T = [  (2*cb)/(3*ca)             (2*sb)/(3*ca)             1/(3*sa);...
    -(cb+sqrt(3)*sb)/(3*ca)    (-sb+sqrt(3)*cb)/(3*ca)   1/(3*sa);...
     (-cb+sqrt(3)*sb)/(3*ca)  -(sb+sqrt(3)*cb)/(3*ca)    1/(3*sa)]; 
K2D = T*K2D;
% 3D Gain 
Q3D = diag([q(3),q(4),q(3),q(4),0,0,q(1),q(2),q(1),q(2)]);
R3D = diag([r,r,r]);
sys = ss(A3D,B3D,C3D,D3D);
[K3D, ~, ~] = lqr(sys,Q3D,R3D);

