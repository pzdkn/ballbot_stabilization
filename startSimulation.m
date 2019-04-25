%-----------------------------------------------------------------
% This function starts the simulation.
%-----------------------------------------------------------------
% input:
%-----------------------------------------------------------------
% return:
%   clientInfo  [3x1] Handle for communication with simulator
%-----------------------------------------------------------------

function clientInfo = startSimulation()
        
  %vrep=remApiSetup();
  vrep=remApi('remoteApi');
  vrep.simxFinish(-1);
  clientInfo.clientID = vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
  if(clientInfo.clientID > -1)
    vrep.simxSynchronous(clientInfo.clientID,true);
    vrep.simxStartSimulation(clientInfo.clientID, vrep.simx_opmode_blocking);
  end
  clientInfo.vrep = vrep;
  clientInfo.mode = vrep.simx_opmode_blocking;
  clientInfo.scripttype = vrep.sim_scripttype_childscript;
  clientInfo.robot = 'BallRobot';

  if(clientInfo.clientID == -1)
    display('Connection to the ball robot not possible.');
  else
    display('Connected successfully to the ball robot.')
  end

end