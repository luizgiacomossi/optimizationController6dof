# Optimization of Force and Torque Bounds for the Flight Control System of a Quadcopter using PSO

This project refers to the work published in the paper entitled "Optimization of Force and Torque Bounds for the Flight Control System of a Quadcopter using PSO" - 19th IEEE Latin American Robotics Symposium - LARS 2022.

## ITA | Aeronautics Institute of Technology 

Authors: Luiz Giacomossi Jr., Angelo Caregnato-Neto and Marcos R. O. A. Maximo || 
Credits: the base code used was created based on the discipline lab work of Prof. Marcos Maximo.

To use the code it is necessary to install: MATLAB 2021_a, Simulink, Aerospace Toolbox and Optimization Toolbox
    - https://www.mathworks.com/products/matlab.html

Files:

    calculatesMse.m                 - Calculates the mean squared error (MSE) of a simulation.
    designController6DoF.m          - Designs a flight controller for a 6DoF UAV analiticly using on the flight system requirements.
    designControllerOptimization.m  - Designs a flight controller for a 6DoF UAV based on the PSO optimization algorithm. The cost of a iteration is calculated by the function costFunPSO" 
    drone_6dof.slx                  - UAV Simulink model
    getDroneDynamics6DoF.m          - get UAV dynamics parameters.
    getRequirements6DoF.m :         - loads the requirements to design the flight controller.
    loadOptimizationResult.m :      - loads results of optimizations for the experiments, pre loads the limits and returns a controller.
    mainNelderMead_NOTUSED.m:       - old code for the optimization, using Nelder Mead algorithm. Not used! it was not effective.
    mainPSO.m: MAIN CODE!           - executes this script to initiate optimization.
    plotComparisonResults6DoF.m:    - Plots comparison between pre and post optimization controllers.
    plotSimulationResults6DoF.m:    - plots detailed graphics of a simulation, used to analise results.
    savePlot.m:                     - saves a plot.
    showResults.m:                  - Loads pre and post optimziation (Pre loads results) controllers and compares results. USE THIS FOR QUICK COMPARISON!
    simulateDrone6DoF.m:            - auxiliary function to execute iteration.
    simulateDrone6DoFExperiment.m:  - simulates a especific experiment

HOW TO USE:

To run the code, open the script "mainPSO.m"! this script will start the optimization of a specific experiment, to select the experiment change the variable "exp" to 'd' for experiment 1, 'f' for experiment 2 or 'g' for experiment 3.

To just load the results already obtained from each experiment, run the script "showResults.m" and again select the desired experiment in the variable "exp".

To plot a the results of a experiment execute the "plotSimulationResults6DoF.m" 

to plot a comparision between 2 simulations, using controllers with different limits, use the "plotComparisonResults6DoF.m:" and pass both simulations on  plotComparisonResults6DoF(simulationPre, simulationPos) 

To load the optimized limits for the flight controller use : controller = loadOptimizationResult(controller, exp)
and pass a controller not yet optimized as a parameter and the experiment.
