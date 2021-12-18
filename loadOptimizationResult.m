function [controller] = loadOptimizationResult(controller, exp)

 % variables are: [fx_min fy_min fz_min fx_max fy_max fz_max tau_x_min tau_z_min tau_x_max tau_z_max]
    % exp 1
 if exp == 'd'
    in = [-0.2441 -1.0565 -0.8279 -0.0271 -0.0012 0.8140 0.8001 1.1752 0.0273 0.0601];

  % exp 2
 elseif exp == 'f'
	% old
    	% in = [-0.5957   -1.2361   -0.0234   -0.0147   -0.0027    0.6482 1.1273    1.2336    0.0752  0.0013] 
    in = [-0.9095 -0.9575 -0.6849 -0.0362 -0.0563 0.9910 0.3963 0.7082 0.0042 0.0442];
  
  % exp 3
 elseif exp == 'g'
    in = [ 0.0000   -0.3910   -1.2072   -0.0316   -0.0011    0.0032    1.0908 1.2338    0.0572    0.0769];
 end
    %% updates limits

 
    controller.x.fMin = in(1);
    controller.x.fMax = in(6);
    
    controller.y.fMin = in(2);
    controller.y.fMax = in(7);

    controller.z.fMin = in(3);
    controller.z.fMax = in(8);
    
    controller.roll.tauMin = in(4);
    controller.roll.tauMax = in(9);
    
    controller.pitch.tauMin = in(4);
    controller.pitch.tauMax = in(9);
    
    controller.yaw.tauMin = in(5);
    controller.yaw.tauMax = in(10);
   
end
