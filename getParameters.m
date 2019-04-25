function m = getModel(parametersSetName)
  
  % Model
  m.g = 9.81;                 % Gravitational acceleration
  switch parameterSetName
      case "ETH"
        m.l = 0.339;            % Height of the center of gravity
        m.mK  = 2.29;           % Mass of the ball
        m.rK = 0.125;           % Radius of the ball
        m.rW = 0.06;            % Radius of the omniwheels
        m.ThetaK = 0.0239;      % Inertia of the ball
        m.ThetaW = 0.00315;     % Inertia of 1 omniwheel + 1 motor
        m.AThetaAWxs = 2.026;   % Inertia of body along x axis
        m.AThetaAWys = 2.025;   % Inertia of body along y axis
        m.AThetaAWzs = 0.092;   % Inertia of body along z axis
        m.mAWs = 12.2;          % Total body mass
           
  end
end