%% Description 
% Run simulation on simulink and on vrep with initial conditions obtained 
% from Vrep. 
%% Obtain Initial Conditions
% Simulation meta data
dt = 0.05; % time step in Vrep [s]
max_t = 20; % sim duration for Vrep and Simulink [s]
m = getParameters("Alex_3D");
% Extract Initial state from Vrep
clientInfo = startSimulation();
% Set Simulation in "motion"
for t=0:dt:0.1
    clientInfo.vrep.simxSynchronousTrigger(clientInfo.clientID); 
end
[theta, dtheta, ball_pos, dphi,target] = getSimState(clientInfo);
% Conversion from single to double
theta = double(theta);
dtheta = double(dtheta);
ball_pos = double(ball_pos);
dphi = double(dphi);
phi = [ ball_pos(2)/m.rK; ball_pos(1)/m.rK];
% initial state for simulink
theta_x0 = theta(1);
theta_y0 = theta(2);
theta_z0 = theta(3);
dtheta_x0 = dtheta(1);
dtheta_y0 = dtheta(2);
dtheta_z0 = dtheta(3);
phi_x0 = phi(1);
phi_y0 = phi(2);
dphi_x0 = dphi(1);
dphi_y0 = dphi(2);
% initial state for vrep
x_0 = [theta(1); dtheta(1); theta(2); dtheta(2); theta(3); dtheta(3);...
        phi(1); dphi(1); phi(2); dphi(2)];

%% Simulink
% Run simulink with given initial state
simOut = sim('BallBot_Plain_Model','SimulationMode','normal',...
            'SaveState','on','StateSaveName','xout',...
            'SaveOutput','on','OutputSaveName','yout',...
 'SaveFormat', 'Dataset');

% Extract simulink output data
outputs = simOut.get('xout');
s_theta_x = outputs{1}.Values;
s_dtheta_x = outputs{2}.Values;
s_theta_y = outputs{3}.Values;
s_dtheta_y = outputs{4}.Values;
s_theta_z = outputs{5}.Values;
s_dtheta_z = outputs{6}.Values;
s_phi_x = outputs{7}.Values;
s_dphi_x = outputs{8}.Values;
s_phi_y = outputs{9}.Values;
s_dphi_y = outputs{10}.Values;
%% Vrep
X = [x_0];
for t=0:dt:max_t
  % Step in simulation
  clientInfo.vrep.simxSynchronousTrigger(clientInfo.clientID); 
  % simulation states
  [theta, dtheta, ball_pos, dphi,target] = getSimState(clientInfo);
  phi = [ ball_pos(2)/m.rK; ball_pos(1)/m.rK];
  x = [theta(1); dtheta(1); theta(2); dtheta(2); theta(3); dtheta(3);...
        phi(1); dphi(1); phi(2); dphi(2)];
  X = [X, x];
end

% Plot data 
t = 0:dt:max_t;
subplot(521)
plot(t,X(1,:));
hold on
plot(s_theta_x);
subplot(522)
plot(t,X(2,:));
hold on
plot(s_dtheta_x);
subplot(523)
plot(t,X(3,:));
hold on
plot(s_theta_y);
sublot(524)
plot(t,X(4,:));
plot(s_dtheta_y);




