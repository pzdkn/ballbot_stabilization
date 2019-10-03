%-----------------------------------------------------------------
% Validate the Quaternion to ZYX intrinsic Euler angle conversion by
% 1.1) Setting orientation via Euler angles converted to quaternions and
% obtaining quaternions that are converted back to Euler angles. Initial
% and obtained angles should be the same.
% 1.2) Using Obtained euler angles to set the orientation again. The
% orientation should remain unchanged.
%-----------------------------------------------------------------
%% Test 1.1
% Inital ZYX angles
ax = pi/16; ay = -pi/4; az = pi/10;
ei = [ax,ay,az];
qi = eul2quat(ax,ay,az,'rzyx');
clientInfo = startSimulation();
simCallScriptFunction(clientInfo, 'set_body_orientation', [], qi , [],'')
% Obtain ZYX angle 
[res retInts qo retStrings retBuffer] = simCallScriptFunction(clientInfo, 'get_body_orientation',[],[],[],'');
eo = quat2eul(qo,'rzyx');
% Test 1 passed. However ay should be contained within (-pi/2, pi/2),
% otherwise input and output angles are not equal. Is that singularity ?
%% Test 1.2
q = eul2quat(eo(1), eo(2), eo(3));
simCallScriptFunction(clientInfo, 'set_body_orientation', [], q , [],'')
% Test 2 passed. However like above there are singularities at 
% ay =-pi/2, pi/2. 
%% OLD
% %-----------------------------------------------------------------
% % We want to validate the angle conversion from XYZ to ZYX, that is
% % the angle conversion from VREP to the Model. We do it by doing the
% % following tests : 
% % 1.1) Assuming zyx2xyz is correct, initialy setting the angles via
% % zyx2xyz, and then obtaining it via xyz2zyx should yield the same
% % initial orientation up to a 2pi modulur ambiguity
% % 1.2) run 1) and use obtained zyx orientation to reset orientation via 
% % zyx2xyz, orientation should preserve
% % 2) Successively set the orientation with euler angles in the ZYX order
% (Adapt accordingly since you are setting wrt absolute frame). Take note
% of the rotated angles and then obtain the converted Euler angle in the
% end. The initial and obtained angles should be the same. 
% %-----------------------------------------------------------------
% 
% %% Test 1.1
% t_x0 = pi/3;
% t_y0 = 0;
% t_z0 = pi/4;
% [alpha, beta, gamma] = zyx2xyz(t_x0, t_y0, t_z0);
% clientInfo = startSimulation();
% simCallScriptFunction(clientInfo, 'set_body_orientation', [], [alpha, beta, gamma], [],'');
% [theta] = getSimState(clientInfo);
% % Test 1 not passed
% %% Test 1.2
% [alpha1, beta1, gamma1] = zyx2xyz(theta(1), theta(2), theta(3));
% simCallScriptFunction(clientInfo, 'set_body_orientation', [], [alpha1, beta1, gamma1], [],'');
% % -> Test 1.2 passed ?!
% %% Test 2
% % Initial ZYX angles
% ax = 0; ay = pi/4; az = pi/4;
% ei = [ax, ay, az];
% qi = eul2quat(ax,ay,az);
% clientInfo = startSimulation();
% % Successively set angles. We are reversing the order from ZYX to XYZ since
% % we are setting wrt to absolute frame ( intrinsic ZYX is extrinsic XYZ).
% simCallScriptFunction(clientInfo, 'set_body_orientation_euler', [], [ax,0,0] , [],'')
% simCallScriptFunction(clientInfo, 'set_body_orientation_euler', [], [0,ay,0] , [],'')
% simCallScriptFunction(clientInfo, 'set_body_orientation_euler', [], [0,0,az] , [],'')
% % Obtain angle
% qo = getSimState(clientInfo);
% eo = quat2eul(qo);