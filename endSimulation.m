%-----------------------------------------------------------------
% This function ends the simulation.
%-----------------------------------------------------------------
% input:
%   clientInfo  [3x1] Handle for communication with simulator
%-----------------------------------------------------------------
% return:
%-----------------------------------------------------------------

function endSimulation(clientInfo)

  clientInfo.vrep.simxStopSimulation(clientInfo.clientID, clientInfo.mode);
  clientInfo.vrep.simxFinish(clientInfo.clientID);

  display('Finished simulation successfully, disconnected to the robot simulation.')

end