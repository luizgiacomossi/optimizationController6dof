
function mse = calculatesMse(simulation)
%this function receives the simulation and calculates the cost of the 
% iteration. COST is calculated using the MSE of each axis

    % reads outputs of simulation
        % euler angles
    outputEuler = simulation.euler.signals.values;
    refEuler = simulation.eulerr.signals.values;
        % x,y,z 
    outputXYZ = simulation.Xg.signals.values;
    refXYZ = simulation.Xr.signals.values;
    
    %disp('Calculating Cost')
    %% calculating cost of iteration
    
    % MSE for x,y and z
    mseXYZ_ = (refXYZ - outputXYZ).^2;
    mseXYZ = sum(mseXYZ_)/length(mseXYZ_);
    mse.mseXYZ =  mseXYZ;
    %mean_mseXYZ = sum(mseXYZ)/length(mseXYZ);
    sumMseXYZ = sum(mseXYZ);
    mse.sumMseXYZ = sumMseXYZ;
    
    % MSE for euler angles, only consider psi in the cost!
    mseEuler_ = (refEuler - outputEuler).^2;
    mseEuler = sum(mseEuler_)/length(mseEuler_);
    mse.mseEuler = mseEuler;
    %mean_mseEuler = sum(mseEuler)/length(mseEuler);
    sumMseEuler = sum(mseEuler(3));
    mse.sumMseEuler = sumMseEuler;
    %cost = mean_mseXYZ + mean_mseEuler;
    cost = sumMseXYZ + sumMseEuler;
    mse.cost = cost;
end
