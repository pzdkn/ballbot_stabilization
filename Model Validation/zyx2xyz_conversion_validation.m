
%-----------------------------------------------------------------
% Validate the ZYX intrinsic Euler angle to Quaternion Conversion by 
% 1) Trying out some simple rotations by successively applying z,y,x 
% elemental rotations and varify qualitatively by observing resulting
% rotations in vrep

%-----------------------------------------------------------------
% Initial ZYX angles
ax = pi/6;
ay = pi/4;
az = pi/3;
q = eul2quat(ax,ay,az);
clientInfo = startSimulation();
simCallScriptFunction(clientInfo, 'set_body_orientation', [], q , [],'')

% Test 1 passed
% zseq = pi/2|pi/3; yseq, xseq = pi|pi/2,pi/4,-pi/2,-pi|
% Tested sequences: zseq,yseq,xseq

%% OLD
% %-----------------------------------------------------------------
% % We want to validate the angle conversion from ZYX to XYZ, that is
% % the angle conversion from the Model to the Vrep. We do it by doing the
% % following tests : 
% % 1) Initially setting the orientation in matlab, and then setting it in
% % VREP. VREP should plot a orientation resulting from ZYX rotation sequence
% %-----------------------------------------------------------------
% 
% %% Test 1
% t_x0 = 0;
% t_y0 = pi/4;
% t_z0 = pi/2;
% [alpha, beta, gamma] = zyx2xyz(t_x0, t_y0, t_z0);
% clientInfo = startSimulation();
% %alpha = t_x0; beta = t_y0; gamma = t_z0;
% simCallScriptFunction(clientInfo, 'set_body_orientation', [], [alpha, beta, gamma], [],'');
% % Test 1 passed