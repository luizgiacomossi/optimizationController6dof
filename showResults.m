clear all
clc

% THIS SCRIPT IS USED TO LOAD AND PRESENT THE RESULTS
% PRE AND POST OPTIMIZATION
% PLEASE SELECT THE EXPERIMENT IN THE VARIABLE "exp"

% open Simulink model
drone_6dof
% obtains parameters describing the dynamics of a 6DoF drone. 
dynamics = getDroneDynamics6DoF();

% getrequirements for the flight control system of the 6DoF drone
requirements = getRequirements6DoF();

% chose experiment to optimize
 % exp 1: 'd' exp 2: 'f' exp 3: 'g'
exp = 'f';

%% Simulation pre-optimization 
% calculates the tracking error and cost pre optimization
controllerPre = designController6DoF(requirements, dynamics);
simulationPre = simulateDrone6DoFExperiment(controllerPre, dynamics, exp);
% calculates cost and errors for pre optimization
msePre = calculatesMse(simulationPre)
disp(sprintf('Cost Pre Optimization: %d',msePre.cost));

%% Simulation post-optimization 
% calculates the tracking error and cost pre optimization

disp('Loading Optimization results using PSO');
controllerPos = loadOptimizationResult(controllerPre, exp);
simulationPos = simulateDrone6DoFExperiment(controllerPos, dynamics, exp);
% calculates cost and errors for post optimization
msePos = calculatesMse(simulationPos)

% plot comparison
plotComparisonResults6DoF(simulationPre, simulationPos)
