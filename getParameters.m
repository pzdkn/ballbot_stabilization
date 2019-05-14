function m = getModel(parameterSetName)
  
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
        m.AThetaAWx = 2.026;   % Inertia of body along x axis
        m.AThetaAWy = 2.025;   % Inertia of body along y axis
        m.AThetaAWz = 0.092;   % Inertia of body along z axis
        m.mAWs = 12.2;          % Total body mass
      case "Alex"
        m.l = 0.55;   
        m.mK = 10.0;   
        m.rK = 0.15;                 
        m.rW = 0.05;                % Radius of the omniwheels    
        m.ThetaK = 9.000e-03;
        m.ThetaW = 1.000e-03;   % Where do I get Inertia of actual wheels instead of virtual wheels ?
        m.AThetaAWx =  5.896e-02/2;
        m.AThetaAWy = 5.896e-02/2;
        m.AThetaAWz = 1.125e-02;
        mA = 26.0;  % Mass of body
        m.mAWs = m.mK + mA;         
  end
end