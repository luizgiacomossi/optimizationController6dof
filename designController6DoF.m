function controller = designController6DoF(requirements, dynamics)
% controller = designController6DoF(requirements, dynamics) designs a
% controller for a 6DoF drone. The structs requirements and dynamics are
% obtained through the functions getRequirements6DoF() and
% getDroneDynamics6DoF(), respectively.
% The output struct controller has the following fields:
% controller.x.Kp: proportional gain of the x controller.
% controller.x.Kv: velocity gain of the x controller.
% controller.x.fMin: minimum force commanded by the x controller.
% controller.x.fMax: maximum force commanded by the x controller.
% controller.y.Kp: proportional gain of the y controller.
% controller.y.Kv: velocity gain of the y controller.
% controller.y.fMin: minimum force commanded by the y controller.
% controller.y.fMax: maximum force commanded by the y controller.
% controller.z.Kp: proportional gain of the z controller.
% controller.z.Kv: velocity gain of the z controller.
% controller.z.fMin: minimum force commanded by the z controller.
% controller.z.fMax: maximum force commanded by the z controller.
% controller.roll.Kp: proportional gain of the roll controller.
% controller.roll.Kv: velocity gain of the roll controller.
% controller.roll.phirMax: maximum reference by the roll controller.
% controller.roll.tauMin: minimum torque commanded by the roll controller.
% controller.roll.tauMax: maximum torque commanded by the roll controller.
% controller.pitch.Kp: proportional gain of the pitch controller.
% controller.pitch.Kv: velocity gain of the pitch controller.
% controller.pitch.thetarMax: maximum reference by the pitch controller.
% controller.pitch.tauMin: minimum torque commanded by the pitch controller.
% controller.pitch.tauMax: maximum torque commanded by the pitch controller.
% controller.yaw.Kp: proportional gain of the pitch controller.
% controller.yaw.Kv: velocity gain of the pitch controller.
% controller.yaw.tauMin: minimum torque commanded by the yaw controller.
% controller.yaw.tauMax: maximum torque commanded by the yaw controller.

    m = dynamics.m;
    % para movimentos de translação wn e xi sao iguais para x,y,z
    xi = requirements.x.xi; 
    wn = requirements.x.wn;
    g = dynamics.g;
    d = dynamics.d;
    k = dynamics.k;
    
 % Position
  % --- X --- 
    controller.x.Kp = m * wn^2 ;
    controller.x.Kv = 2 * m * xi * wn ;
    
    %controller.x.fMin = param.optimization;
    %controller.x.fMax = param.optimization;
    
    controller.x.fMin = -0.5 * m * g;
    controller.x.fMax = +0.5 * m * g;
  % --- Y --- 
    controller.y.Kp =  m * wn^2;
    controller.y.Kv = 2 * m * xi * wn;
    
    controller.y.fMin = -0.5 * m * g;
    controller.y.fMax = +0.5 * m * g;
  % --- Z --- 
    controller.z.Kp =  m * wn^2;
    controller.z.Kv =  2 * m * xi * wn;
    
    controller.z.fMin = 0.1 * m * g;
    controller.z.fMax = 2 * m * g;
    
    % req para roll/pitch/yaw sao iguais
    wn = requirements.roll.wn ;
    xi = requirements.roll.xi;
    
    controller.roll.Kp = dynamics.J(1,1) * wn^2 ; 
    controller.roll.Kv = 2 * dynamics.J(1,1) * xi * wn;
    controller.roll.phirMax = 30 * pi/180;
    controller.roll.tauMin = -4 * m * g * d;
    controller.roll.tauMax = +4 * m * g * d;

    controller.pitch.Kp = dynamics.J(2,2) * wn^2;
    controller.pitch.Kv =  2 * dynamics.J(2,2) * xi * wn;
    controller.pitch.thetarMax = 30 * pi/180;
    controller.pitch.tauMin = -4 * m * g * d;
    controller.pitch.tauMax = +4 * m * g * d;

    controller.yaw.Kp = dynamics.J(3,3) * wn^2;
    controller.yaw.Kv = 2 * dynamics.J(3,3) * xi * wn;
    controller.yaw.tauMin = -4 * m * g * k;
    controller.yaw.tauMax = +4 * m * g * k;

end