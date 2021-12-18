# 6DoF UAV Flight Control Saturation Threshold Optimization
This project refers to the final assignment of the subject "Planning and Control for Mobile Robotics" CM-202 
ITA | Aeronautics Institute of Technology 

Author: Luiz Giacomossi Jr. || 
Credits: the base code used was created based on the discipline lab work of Prof. Marcos Maximo.

To use the code it is necessary: MATLAB 2021_a

Files:

    calculatesMse.m                 - Calculates the mean squared error (MSE) of a simulation.
    designController6DoF.m          - Designs a flight controller for a 6DoF UAV analiticly using on the flight system requirements.
    designControllerOptimization.m  - Designs a flight controller for a 6DoF UAV based on the PSO optimization algorithm. 
    drone_6dof.slx                  - UAV Simulink model
    getDroneDynamics6DoF.m          - get UAV dynamics parameters.
    getRequirements6DoF.m :         - loads the requirements to design the flight controller.
    loadOptimizationResult.m :      - loads results of optimizations for the experiments.
    mainNelderMead_NOTUSED.m:       - old code for the optimization, using Nelder Mead algorithm. Not used! it was not effective.
    mainPSO.m: MAIN CODE!           - executes this script to initiate optimization.
    plotComparisonResults6DoF.m:    - Plots comparison between pre and post optimization controllers.
    plotSimulationResults6DoF.m:    - plots detailed graphics of a simulation, used to analise results.
    savePlot.m:                     - saves a plot.
    showResults.m:                  - Loads pre and post optimziation (Pre loads results) controllers and compares results. USE THIS FOR QUICK COMPARISON!
    simulateDrone6DoF.m:            - auxiliary function to execute iteration.
    simulateDrone6DoFExperiment.m:  - simulates a especific experiment
