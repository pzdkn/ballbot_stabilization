function m = getParameters(parameterSetName)
  
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
      case "Alex_3D"
        m.l = 0.45;   
        m.mK = 10; 
        m.mA = 20.0;  % Mass of body
        m.rK = 0.15;                 
        m.rW = 0.05;                % Radius of the omniwheels    
        m.ThetaK = 9.000e-03*m.mK;
        m.ThetaW = 2.000e-03;   % ThetaOW +ThetaM (Wheel + Motor)
        m.AThetaAWx =  5.896e-02;
        % Steiner : Inertia is given in reference to ball center
        m.AThetaAWx = m.AThetaAWx*m.mA  + m.mA*m.l^2;
        m.AThetaAWy = 5.896e-02;
        % Steiner 
        m.AThetaAWy = m.AThetaAWy*m.mA + m.mA*m.l^2;
        m.AThetaAWz = 1.125e-02*m.mA;
        m.mAWs = m.mK + m.mA;   
      case "Alex_2D"
        m.mK = 10.0;                % Mass of the ball
        m.mW = 0;                   % Mass of the virtual actuating wheel
        m.mA = 26.0;                % Mass of the body
        m.rK = 0.15;                % Radius of the ball
        m.rW = 0.05;                % Radius of the omniwheels
        m.rA = 0.15;                % Radius of the body (cylinder)
        m.l = 0.55;                 % Height of the center of gravity
        m.ThetaOW = 1.000e-03;      % Inertia of omniwheels
        m.ThetaM = 1.000e-03;       % Inertia of motor rotor
        m.i = 1;                    % Gear ratio
        m.alpha = pi/4;             % Working angle of omni wheels
        m.ThetaW = 3/2*cos(m.alpha)^2*(m.ThetaOW + m.i^2*m.ThetaM);   % Inertia of the actuating wheel in the yz-/xz-plane
        m.ThetaWxy = 3*(m.ThetaOW + m.i^2*m.ThetaM);                  % Inertia of the actuating wheel in the xy-plane
        m.ThetaK = 9.000e-03;   %2/3*m.mK*m.rK^2;                     % Inertia of the ball
        m.ThetaA = 5.896e-02;   %m.mA*m.l^2;                          % Inertia of the body in the yz-/xz-plane
        m.ThetaAxy = 1.125e-02; %1/2*(m.mA + m.mW)*m.rA^2;            % Inertia of the body in the xy-plane
        m.mTot = m.mK + m.mW + m.mA;                                  % Total mass of robot
        m.rTot = m.rK + m.rW;
        m.gamma = m.l*m.mA + m.rTot*m.mW;

        % constants required for simplified model
        m.c_1 = m.mTot*m.rK^2 + m.ThetaK + (m.rK/m.rW)^2*m.ThetaW;
        m.c_2 = -(m.rK/m.rW^2)*m.rTot*m.ThetaW;
        m.c_3 = m.rTot^2/m.rW^2*m.ThetaW + m.ThetaA + m.mA*m.l^2 + m.mW*m.rTot^2;
        m.c_4 = m.gamma*m.rK;
        m.c_5 = m.rK/m.rW;
        m.c_m = (m.ThetaAxy*m.rW)/(m.rK*sin(m.alpha)) + (m.ThetaWxy*m.rK*sin(m.alpha))/(m.rW); 
  end
end