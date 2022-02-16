clc;
clear;
close all;
q = [10, 50, 10, 50, 5, 20, 20, 100, 20, 100];
r = [100, 100, 100];
rho = 2;
max_t = 500; %[s]
dt = 0.05; % time step in [s]
%% Run 2D Model
[O2D] = run2dController(max_t,dt);
t = 0:dt:max_t;
% plot states
figure()
plot(t,O2D.X(1,:),'b')
hold on
yline(O2D.T(1),'r')
title('2D Body tilt around x-axis')
xlabel('[s]')
ylabel('[rad]')
figure()
plot(t,O2D.X(5,:),'b')
hold on
yline(O2D.T(5),'r')
title('2D Body tilt around z-axis')
xlabel('[s]')
ylabel('[rad]')
figure()
plot(t,O2D.X(7,:),'b')
hold on
yline(O2D.T(7),'r')
title('2D Ball roll angle around x-axis')
xlabel('[s]')
ylabel('[rad]')
figure()
plot(t,O2D.U(1,:),'r',t,O2D.U(2,:),'g',t,O2D.U(3,:),'b');
title('2D Control output')
xlabel('[s]')
ylabel('[Nm]')
%% Run 3D Model
[O3D] = run3dController(q,r,rho,max_t,dt);
t = 0:dt:max_t;
% plot states
figure()
plot(t,O3D.X(1,:),'b')
hold on
yline(O3D.T(1),'r')
title('3D Body tilt around x-axis')
xlabel('[s]')
ylabel('[rad]')
figure()
plot(t,O3D.X(5,:),'b')
hold on
yline(O3D.T(5),'r')
title('3D Body tilt around z-axis')
xlabel('[s]')
ylabel('[rad]')
figure()
plot(t,O3D.X(7,:),'b')
hold on
yline(O3D.T(7),'r')
title('3D Ball roll angle around x-axis')
xlabel('[s]')
ylabel('[rad]')
figure()
plot(t,O3D.U(1,:),'r',t,O3D.U(2,:),'g',t,O3D.U(3,:),'b');
title('3D Control output')
xlabel('[s]')
ylabel('[Nm]')
