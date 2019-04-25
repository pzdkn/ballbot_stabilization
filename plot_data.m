% This scripts plots all collected data
t = 0:dt:max_t;

subplot(3,5,1)
plot(t, P(2,:), t, DP(2,:))
xlim([0, max_t]);
xlabel("t");
ylabel("y");
subplot(3,5,2)
plot(t, YZ(1,:), t, DS(1,:))
xlim([0, max_t]);
xlabel("t");
ylabel("phi_x (yz-Plane Ball)");
subplot(3,5,3)
plot(t, YZ(3,:))
xlim([0, max_t]);
xlabel("t");
ylabel("theta_x (yz-Plane Body)");
subplot(3,5,4)
plot(t, YZ(4,:))
xlim([0, max_t]);
xlabel("t");
ylabel("dtheta_x (yz-Plane Body)");
subplot(3,5,5)
plot(t, U(1,:))
xlim([0, max_t]);
xlabel("t");
ylabel("u_x");

subplot(3,5,6)
plot(t, P(1,:), t, DP(1,:))
xlim([0, max_t]);
xlabel("t");
ylabel("x");
subplot(3,5,7)
plot(t, XZ(1,:), t, DS(2,:))
xlim([0, max_t]);
xlabel("t");
ylabel("phi_y (xz-Plane Ball)");
subplot(3,5,8)
plot(t, XZ(3,:))
xlim([0, max_t]);
xlabel("t");
ylabel("theta_y (xz-Plane Body)");
subplot(3,5,9)
plot(t, XZ(4,:))
xlim([0, max_t]);
xlabel("t");
ylabel("dtheta_y (xz-Plane Body)");
subplot(3,5,10)
plot(t, U(2,:))
xlim([0, max_t]);
xlabel("t");
ylabel("u_y");

%subplot(3,5,11)
%plot(t, E(1,:), t, E(2,:))
%xlim([0, max_t]);
%xlabel("t");
%ylabel("e");
subplot(3,5,12)
plot(t, XY(1,:))
xlim([0, max_t]);
xlabel("t");
ylabel("phi_z (xy-Plane Ball)");
subplot(3,5,13)
plot(t, XY(3,:), t, DS(3,:))
xlim([0, max_t]);
xlabel("t");
ylabel("theta_z (xy-Plane Body)");
subplot(3,5,14)
plot(t, XY(4,:))
xlim([0, max_t]);
xlabel("t");
ylabel("dtheta_z (xy-Plane Body)");
subplot(3,5,15)
plot(t, U(3,:))
xlim([0, max_t]);
xlabel("t");
ylabel("u_z");