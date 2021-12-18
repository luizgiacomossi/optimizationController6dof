clear all
clc

% THIS SCRIPT IS USED TO OPTIMIZE THE FLIGHT CONTROLLER LIMITS
% SELECT EXPERIMENT IN THE VARIABLE "exp"

% options for ploting the optimizations 
options = optimset('Display','iter','PlotFcns',@optimplotfval);

% open Simulink model
drone_6dof

% obtains parameters describing the dynamics of a 6DoF drone. 
    % m: mass.
    % J: inertia matrix.
    % l: distance between the center of mass and each propeller.
    % d: distance between the center of mass and each propeller along the x or y body axis.
    % g: acceleration of gravity.
    % kf: thrust force constant.
    % ktau: reaction torque constant.
    % k: the ratio ktau / kf.
    % wMin: minimum velocity of each propeller.
    % wMax: maximum velocity of each propeller.
    % Gamma: matrix to transform from propeller forces
dynamics = getDroneDynamics6DoF();

% getrequirements for the flight control system of the 6DoF drone
    % x y z .wn: natural frequency of the x channel.
    % x y z .xi: damping factor of the x channel.
    % roll pitch yaw .wn: natural frequency of the roll channel.
    % roll pitch yaw .xi : damping factor of the roll channel.
requirements = getRequirements6DoF();

% chose experiment to optimize
 % exp 1: 'd' exp 2: 'f' exp 3: 'g'
exp = 'f';

%% Simulation pre-optimization 
% calculates the tracking error and cost pre optimization
controllerPre = designController6DoF(requirements, dynamics);
simulationPre = simulateDrone6DoFExperiment(controllerPre, dynamics, exp);
%plotSimulationResults6DoF(simulationPre);

% calculates the tracking error and cost pre optimization
msePre = calculatesMse(simulationPre)

disp(sprintf('Cost Pre Optimization: %d',msePre.cost));
    
disp('Begining Optimization using PSO');
% design controller based on dynamics and requiremets
% and run optimization on  3D experiment - exp1:'d', exp2:'f', exp3:'g'
outputOptimizator = designControllerOptimization(requirements, dynamics, exp);

% print optimization result
disp('Optimization Finished');
disp(sprintf('Best Cost post Optimizaion: %d',outputOptimizator.bestcost))
outputOptimizator.bestcost

%% updates controller with new limits
% executes simulation with optimized controller and plot results
controller = controllerPre;

controller.x.fMin  =  outputOptimizator.Limits(1);
controller.y.fMin  = outputOptimizator.Limits(2);

controller.x.fMax =   outputOptimizator.Limits(6);
controller.y.fMax =   outputOptimizator.Limits(7);

controller.z.fMin  =   outputOptimizator.Limits(3);
controller.z.fMax  =   outputOptimizator.Limits(8);

controller.roll.tauMin =    outputOptimizator.Limits(4) ;
controller.roll.tauMax =    outputOptimizator.Limits(9) ;

controller.pitch.tauMin = controller.roll.tauMin;
controller.pitch.tauMax = controller.roll.tauMax;

controller.yaw.tauMin  =     outputOptimizator.Limits(5);
controller.yaw.tauMax  =     outputOptimizator.Limits(10) ;

% Print limits found 
disp('LIMITS FOUND:')
text = 'x.fMin: %f  x.fMax: %f \n';
fprintf(text,controller.x.fMin, controller.x.fMax)
text = 'y.fMin: %f  y.fMax: %f \n';
fprintf(text,controller.y.fMin, controller.y.fMax)
text = 'z.fMin: %f  z.fMax: %f \n';
fprintf(text,controller.z.fMin, controller.z.fMax)
text = 'x.tauMin: %f  x.tauMax: %f \n';
fprintf(text,controller.roll.tauMin,controller.roll.tauMax)
text = 'y.tauMin: %f  y.tauMax: %f \n';
fprintf(text,controller.pitch.tauMin, controller.pitch.tauMax)
text = 'z.tauMin: %f  z.tauMax: %f \n';
fprintf(text,controller.yaw.tauMin, controller.yaw.tauMax )

% simulates using new limits
simulationPos = simulateDrone6DoFExperiment(controller, dynamics, exp);
%plotSimulationResults6DoF(simulationPos);
msePre
% calculates cost and errors
msePos = calculatesMse(simulationPos)
% plot comparision pre and post optimization
plotComparisonResults6DoF(simulationPre, simulationPos)


