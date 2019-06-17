clc;
clear;
close all;
q = [10, 500, 10, 500, 1, 50, 20, 100, 20, 100];
r = [200, 200, 200];
rho = 1;
[U_2d, dt, max_t] = run2dController();
%[U_3d, dt, max_t] = run3dController(q,r,rho);
subplot(121)
hold on
t = 0:dt:max_t;
plot(t,U_2d(1,:),"b")
plot(t,U_2d(2,:),"g")
plot(t,U_2d(3,:),"y")
hold off
title("2D Control Output")
% subplot(122)
% hold on
% plot(t,U_3d(1,:),"b")
% plot(t,U_3d(2,:),"g")
% plot(t,U_3d(3,:),"y")
% hold off
% title("3D Control Output")
