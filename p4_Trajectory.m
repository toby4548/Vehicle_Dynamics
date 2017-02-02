%
%  Cornering Vehicle Dynamics Simulation
%
%  Created by I.Chen and Y.Huang in 2015.
%  Copyright (c) 2015 All rights reserved.
% 


%Vehicle Trajectory
theta_n=theta_c+r_c*dt;
x_n=x_c+u_c*cos(theta_n)*dt-v_c*sin(theta_n)*dt;
y_n=y_c+u_c*sin(theta_n)*dt+v_c*cos(theta_n)*dt;

