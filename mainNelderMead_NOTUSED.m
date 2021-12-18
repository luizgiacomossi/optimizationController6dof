clear all
clc
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

% design controller based on dynamics and requiremets
controller = designController6DoF(requirements, dynamics);

% simulates 3D experiment - 'a','b','c' or 'd'
%simulation = simulateDrone6DoFExperiment(controller, dynamics, 'd')

% optimization of cost function using Nelder-Mead
%c1 = [controller.x.Kp controller.x.Kv controller.x.fMin controller.x.fMax];
%c2 = [controller.y.Kp controller.y.Kv controller.y.fMin controller.y.fMax];
%c3 = [controller.z.Kp controller.z.Kv controller.z.fMin controller.z.fMax];

%c4 = [controller.roll.Kp  controller.roll.Kv  controller.roll.tauMin  controller.roll.tauMax];
%c5 = [controller.pitch.Kp controller.pitch.Kv controller.pitch.tauMin controller.pitch.tauMax];
%c6 = [controller.yaw.Kp   controller.yaw.Kv   controller.yaw.tauMin   controller.yaw.tauMax];

c1 = [controller.x.fMin controller.x.fMax];
%c2 = [controller.y.fMin controller.y.fMax];
c3 = [controller.z.fMin controller.z.fMax];

c4 = [controller.roll.tauMin  controller.roll.tauMax];
%c5 = [controller.pitch.tauMin controller.pitch.tauMax];
c6 = [controller.yaw.tauMin   controller.yaw.tauMax];

c=[c1;c3 
   c4;c6]
disp('Otimização')
a = fminsearch(@cost_function,c,options)
disp('sai Otimização')