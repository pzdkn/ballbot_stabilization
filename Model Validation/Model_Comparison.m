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
%%
