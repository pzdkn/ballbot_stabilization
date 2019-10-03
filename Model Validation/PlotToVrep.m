% Set orientation and position in Vrep according to solution of ode in
% matlab
%% Obtain Model Parameters
m = getParameters("Alex_3D");
l = 0.75 - m.rK; %0.4 because the cylinder's height in vrep is 0.8, we plot the cylinders center point
%% Define Initial states
t_x0 = pi/4;
dt_x0 = 0;
t_y0 = 0;
dt_y0 = 0;
t_z0 =  0;
dt_z0 = 0;
p_x0 = 2*pi;
dp_x0 =0;
p_y0 = 0;
dp_y0 = 0;
x0 = [t_x0; dt_x0; t_y0; dt_y0; t_z0; dt_z0; p_x0; dp_x0; p_y0; dp_y0];
%% Start Vrep
clientInfo = startSimulation();
[~, ~, ball_pos, ~, ~] = getSimState(clientInfo);
ball_height = 0.15;
% set initial states in Vrep
theta = [t_x0; t_y0; t_z0];
ball_pos = [m.rK*p_y0; - m.rK*p_x0; ball_height];
body_pos = [m.rK*p_y0 + (sin(t_z0)*sin(t_x0) + cos(t_z0)*sin(t_y0)*cos(t_x0))*l; ...
    - m.rK*p_x0 + (-cos(t_z0)*sin(t_x0) + sin(t_z0)*sin(t_y0)*cos(t_x0))*l;...
    m.rK + cos(t_y0)*cos(t_x0)*l];
q = eul2quat(t_x0, t_y0, t_z0,'rzyx');
%%TODO : need to add ball height to body height, but dont incorporate body
%%angles into it, deal with it seperately !
simCallScriptFunction(clientInfo, 'set_body_orientation', [], q , [],'');
simCallScriptFunction(clientInfo, 'set_body_position', [], body_pos, [],'');
simCallScriptFunction(clientInfo, 'set_ball_position', [], ball_pos, [],'');
clientInfo.vrep.simxSynchronousTrigger(clientInfo.clientID); 
%% Solve ODE
t_end = 20;
t_start = 0;
dt = 0.05;
ode_fcn = @(t,x) nonlinear_fcn_v3_eth(x,[0,0,0]);
[t,x] = ode45(ode_fcn,t_start:dt:t_end, x0);
%% Plot Solutions to Vrep
for i=1:length(x)
    q = eul2quat(x(i,1), x(i,3), x(i,5),'rzyx');
    ball_pos = [x(i,9)*m.rK; - x(i,7)*m.rK; ball_height];
    body_pos = [m.rK*x(i,9) + (sin(x(i,5))*sin(x(i,1)) + cos(x(i,5))*sin(x(i,3))*cos(x(i,1)))*l; ...
    - m.rK*x(i,7) + (-cos(x(i,5))*sin(x(i,1)) + sin(x(i,5))*sin(x(i,3))*cos(x(i,1)))*l;...
    cos(x(i,3))*cos(x(i,1))*l];
    simCallScriptFunction(clientInfo, 'set_body_orientation', [], q, [],'');
    simCallScriptFunction(clientInfo, 'set_ball_position', [], ball_pos, [],'');
    simCallScriptFunction(clientInfo, 'set_body_position', [], body_pos, [],'');
    clientInfo.vrep.simxSynchronousTrigger(clientInfo.clientID); 
end
endSimulation(clientInfo)
