% This scripts plots all collected data
t = 0:dt:max_t;

%plotting y
subplot(3,5,1)
plot(t, POS(2,:), t, TARGET_POS(2,:))
xlim([0, max_t]);
xlabel("t");
ylabel("y");
%plotting phi_x_err
subplot(3,5,2)
plot(t, X(7,:))
hold on;
plot([0,max_t], [0,0])
xlim([0, max_t]);
xlabel("t");
ylabel("phi_x");
%plotting theta_y
subplot(3,5,3)
plot(t, X(1,:))
xlim([0, max_t]);
xlabel("t");
ylabel("theta_x");
%plotting dtheta_x
subplot(3,5,4)
plot(t, X(2,:))
xlim([0, max_t]);
xlabel("t");
ylabel("dtheta_x");
%plotting u1
subplot(3,5,5)
plot(t, U(1,:))
xlim([0, max_t]);
xlabel("t");
ylabel("u_1");
%plotting x
subplot(3,5,6)
plot(t, POS(1,:), t, TARGET_POS(1,:))
xlim([0, max_t]);
xlabel("t");
ylabel("x");
subplot(3,5,7)
%plotting phi_y_err
plot(t, X(8,:))
hold on;
plot([0,max_t], [0,0])
xlim([0, max_t]);
xlabel("t");
ylabel("phi_y");
subplot(3,5,8)
%plotting theta_y
plot(t, X(3,:))
xlim([0, max_t]);
xlabel("t");
ylabel("theta_y");
%plotting dtheta_y
subplot(3,5,9)
plot(t, X(4,:))
xlim([0, max_t]);
xlabel("t");
ylabel("dtheta_y");
%plotting u2
subplot(3,5,10)
plot(t, U(2,:))
xlim([0, max_t]);
xlabel("t");
ylabel("u_2");

%subplot(3,5,11)
%plot(t, E(1,:), t, E(2,:))
%xlim([0, max_t]);
%xlabel("t");
%ylabel("e");

%
%subplot(3,5,12)
%plot(t, XY(1,:))
%xlim([0, max_t]);
%xlabel("t");
%ylabel("phi_z (xy-Plane Ball)");

%plotting theta_z_err
subplot(3,5,13)
plot(t, X(5,:))
hold on;
plot([0,max_t], [0,0])
xlim([0, max_t]);
xlabel("t");
ylabel("theta_z ");
%plotting dtheta_z
subplot(3,5,14)
plot(t, X(6,:))
xlim([0, max_t]);
xlabel("t");
ylabel("dtheta_z");
%plotting u3
subplot(3,5,15)
plot(t, U(3,:))
xlim([0, max_t]);
xlabel("t");
ylabel("u_3");