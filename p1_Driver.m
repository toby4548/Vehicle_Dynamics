%
%  Cornering Vehicle Dynamics Simulation
%
%  Created by I.Chen and Y.Huang in 2015.
%  Copyright (c) 2015 All rights reserved.
% 
switch cyc_type
    case 1 % (1) split road mu
        delta_f_c = 0;
        %delta_f_c=
        % Throttle Control (PID control)
v_error_d = v_error_p-(v_target*kmh2ms-u_c);
v_error_i = v_error_i+v_error_p;%accumulate all the error before
v_error_p = (v_target*kmh2ms-u_c);%find the error between desire speed and true speed

    case 2 % (2) constant cornering
        
        % Conering Control (PID control)
R_error_d = R_error_p-(R_target-R_c);
R_error_i = R_error_i+R_error_p;%accumulate all the error before
R_error_p = (R_target-R_c);%find the error between desire speed and true speed
        
        % Steering Angle
        if t<turning_time
            delta_f_c = 0; % + : trun left; - : turn right
        else
            delta_f_c = delta_f_ctrl*deg2rad;
            %{
            if R_c<30
            delta_f_c = delta_f_ctrl*deg2rad+0.5*R_error_p +0*R_error_i+0*v_error_d;
            else
            delta_f_c = delta_f_ctrl*deg2rad;
            end
            %}
        end
        
end

% Throttle Control (PID control)
PID_kp=30000;
PID_ki=300;
PID_kd=0;
v_error_d = v_error_p-(v_target*kmh2ms-u_c);
v_error_i = v_error_i+v_error_p;%accumulate all the error before
v_error_p = (v_target*kmh2ms-u_c);%find the error between desire speed and true speed


switch cyc_type
    case 1 % (1) split road mu
        %T_in_c =23.1488;
        T_in_c =100;
        T_in_c=T_totaltc_c;
    case 2 % (2) constant cornering
        T_in_c = T_in_c_initial +PID_kp*v_error_p+PID_ki*v_error_i+PID_kd*v_error_d;

end
