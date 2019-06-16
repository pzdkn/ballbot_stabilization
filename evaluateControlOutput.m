q = [100, 10, 100, 10, 5, 5, 50, 5, 50, 5];
r = [100, 100, 100];
rho = 1;
[U_2d, t] = run2dController();
[U_3d, t] = run3dController();
subplot(121)
plot(t,U_2d')
title("2D Control Output")
subplot(122)
plot(t, U_3d')
title("3D Control Output")