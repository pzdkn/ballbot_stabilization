%-----------------------------------------------------------------
% Wrapper to call simxCallScriptFunction
%-----------------------------------------------------------------
% input:
%   clientInfo      [3x1] Handle for communication with simulator
%   functionName    [1x1] String of function name to call 
%   inInts
%   inFloats
%   inStrings
%   inBuffer
%-----------------------------------------------------------------
% return:
%   [res retInts retFloats retStrings retBuffer]  Return data from Simulator
%-----------------------------------------------------------------

function [res retInts retFloats retStrings retBuffer] = simCallScriptFunction(clientInfo, functionName, inInts ...
    ,inFloats,inStrings,inBuffer)
    
  [res retInts retFloats retStrings retBuffer] = clientInfo.vrep.simxCallScriptFunction(...
        clientInfo.clientID,...
        clientInfo.robot,...
        clientInfo.scripttype,...
        functionName,...
        inInts, inFloats, inStrings, inBuffer,...
        clientInfo.mode);
  
end