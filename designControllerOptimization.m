function optimized_out = designControllerOptimization(requirements, dynamics, experiment)
% controller = designControllerOptimization(requirements, servo) designs 
% the speed compensator based on requirements and the servo's parameters. 
% This function uses an approach based on optimization for the design.
% The structs requirements and servo follow the format returned by the 
% functions getRequirements() and getServoParams(), respectively. The
% struct controller has the following fields:
% controller.Kp: proportional gain.
% controller.Ki: integrative gain.
% controller.T: sample time.

    % Using analytic solution as initial guess
    controllerAnalytic = designController6DoF(requirements, dynamics);

    % Initial guess
    fRotorMax = 2.0 * dynamics.m * dynamics.g;
    fRotorMin = 0;
    tmax = 2*dynamics.l*dynamics.m*dynamics.g;
    %c1 = [-fRotorMax +fRotorMax];
    
    % shows base limits as the reference
    c1 = [controllerAnalytic.x.fMin controllerAnalytic.x.fMax];
    %c2 = [controller.y.fMin controller.y.fMax];
    c3 = [controllerAnalytic.z.fMin controllerAnalytic.z.fMax];
    %c3 = [-fRotorMax +fRotorMax];
    c4 = [controllerAnalytic.roll.tauMin  controllerAnalytic.roll.tauMax];    
    %c5 = [controller.pitch.tauMin controller.pitch.tauMax];
    c6 = [controllerAnalytic.yaw.tauMin   controllerAnalytic.yaw.tauMax];
    BoundsExperimentalReference = [c1;c3;c4;c6]

    
    %% NELDER MEAD - Not used anymore
    % Options to display the optimization progress
    % options = optimset('Display','iter','PlotFcns',@optimplotfval);

    % Optimization
    %limits = fminsearch(@(x) cost_fun(requirements, dynamics, controllerAnalytic, x), x0, options);
    
    %%  PSO
    rng default  % For reproducibility
    %% Defining the number of variables to be optimized and its bounds
    nvars = 10;
    % variables are: [fx_min fy_min fz_min fx_max fy_max fz_max tau_x_min tau_z_min tau_x_max tau_z_max]
    % lower bound
    %lb = [controllerAnalytic.x.fMin*4, controllerAnalytic.y.fMin*4, controllerAnalytic.z.fMin*-2, controllerAnalytic.roll.tauMin*4, controllerAnalytic.yaw.tauMin*4,...
        %controllerAnalytic.x.fMax*0.5, controllerAnalytic.y.fMax*0.5,controllerAnalytic.z.fMax*0.5, controllerAnalytic.roll.tauMax*0.5, controllerAnalytic.yaw.tauMax*0.5];
    % upper bound
    %ub = [controllerAnalytic.x.fMin*0.5, controllerAnalytic.y.fMin*0.5, controllerAnalytic.z.fMin*4, controllerAnalytic.roll.tauMin*0.5, controllerAnalytic.yaw.tauMin,...
        %controllerAnalytic.x.fMax*4, controllerAnalytic.y.fMax*4, controllerAnalytic.z.fMax*4, controllerAnalytic.roll.tauMax*4, controllerAnalytic.yaw.tauMax*4];
   
   %% Defining the number of variables to be optimized and its bounds
    % tmax = 2*l*m*g
    % variables are: [fx_min fy_min fz_min fx_max fy_max fz_max tau_x_min tau_z_min tau_x_max tau_z_max]
    nvars = 10;
    lb = [-fRotorMax, -fRotorMax, -fRotorMax, tmax*-1, tmax*-1,...
          0.00001,  0.00001,  0.00001, 0, 0];
    
    ub = [ 0.00001, 0.00001,  0.00001, 0, 0,...
          fRotorMax, fRotorMax, fRotorMax, tmax, tmax];
   
   %%
    % defines cost function
    fun = @(x) costFunPSO(requirements, dynamics, controllerAnalytic, experiment,x);
    % PSO parameters
     % ,'HybridFcn',@fmincon
    options = optimoptions('particleswarm','SwarmSize',20, 'Display', 'iter', 'PlotFcn','pswplotbestf');
    % run optimization 
    [x,fval,exitflag,output] = particleswarm(fun,nvars,lb,ub,options)
    
    optimized_out.controller_pre = controllerAnalytic;
    optimized_out.Limits = x;
    optimized_out.bestcost = fval;

end


function cost = cost_fun(requirements, dynamics, controller, in)
%COST_FUNCTION OLD FUNCTION FOR NELDER MEAD OPTIMIZATION- NOT USED !!
%   Detailed explanation goes here
    %disp('Loading parameters')
    % simulates 3D experiment - 'a','b','c' or 'd'
    %dynamics = getDroneDynamics6DoF();
    %requirements = getRequirements6DoF();
    %controller = designController6DoF(requirements, dynamics);
    in;
    %disp('assigin inputs')
    %% PARSING INPUTS
    B = num2cell(in(1,:));
    [controller.x.fMin,controller.x.fMax] = B{:};
    
    %B = num2cell(in(2,:));
    [controller.y.fMin,controller.y.fMax] = B{:};
    
    B = num2cell(in(2,:));
    [controller.z.fMin,controller.z.fMax] = B{:};
    
    B = num2cell(in(3,:));
    [controller.roll.tauMin,controller.roll.tauMax] = B{:};
    
    %B = num2cell(in(5,:));
    [controller.pitch.tauMin,controller.pitch.tauMax] = B{:};
    
    B = num2cell(in(4,:));
    [controller.yaw.tauMin,controller.yaw.tauMax] = B{:};
    
    %disp('executing simulaion')
    simulation = simulateDrone6DoFExperiment(controller, dynamics, 'e');
    
    saida_euler = simulation.euler.signals.values;
    ref_euler = simulation.eulerr.signals.values;
    saida_xyz   = simulation.Xg.signals.values;
    ref_xyz = simulation.Xr.signals.values;
    
    veloc = simulation.Vg.signals.values;
    %disp('Calculating Cost')
    %% calculating cost of iteration
    %cost = sum((saida_euler - ref_euler).^2 + (saida_xyz - ref_xyz).^2);
    
    
    mse_xyz_ = (ref_xyz - saida_xyz).^2;
    mse_xyz = sum(mse_xyz_)/length(mse_xyz_);
    %mean_mse_xyz = sum(mse_xyz)/length(mse_xyz);
    sum_mse_xyz = sum(mse_xyz);
    
    mse_euler_ = (ref_euler - saida_euler).^2;
    mse_euler = sum(mse_euler_)/length(mse_euler_);
    %mean_mse_euler = sum(mse_euler)/length(mse_euler);
    sum_mse_euler = sum(mse_euler);
    
    %cost = mean_mse_xyz + mean_mse_euler;
    cost = sum_mse_xyz + sum_mse_euler;


    %disp('end of iteration')
end

function cost = costFunPSO(requirements, dynamics, controller, experiment, in)
%COST_FUNCTION: THIS DEFINES THE COST OF EACH ITERATION FOR THE PSO ALG.
    %disp('Loading parameters')
    in;
    %disp('updates inputs')
     %% PARSING INPUTS FOR 10
     
    controller.x.fMin = in(1);
    controller.x.fMax = in(6);
    
    %B = num2cell(in(2,:));
    controller.y.fMin = in(2);
    controller.y.fMax = in(7);

    %B = num2cell(in(2,:));
    controller.z.fMin = in(3);
    controller.z.fMax = in(8);
    
    %B = num2cell(in(3,:));
    controller.roll.tauMin = in(4);
    controller.roll.tauMax = in(9);
    
    %B = num2cell(in(5,:));
    controller.pitch.tauMin = in(4);
    controller.pitch.tauMax = in(9);
    
    %B = num2cell(in(4,:));
    controller.yaw.tauMin = in(5);
    controller.yaw.tauMax = in(10);
     
    % Executes simulation for particle 
    simulation = simulateDrone6DoFExperiment(controller, dynamics, experiment);
    
    %% calculating cost of iteration
    mse = calculatesMse(simulation);
    cost = mse.cost;    
    
    %disp('end of iteration')
end

