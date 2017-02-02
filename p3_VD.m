%
%  Cornering Vehicle Dynamics Simulation
%
%  Created by I.Chen and Y.Huang in 2015.
%  Copyright (c) 2015 All rights reserved.
% 


% Vehicle Dynamics

% Rolling Resistance
R_r1L_c = F_n_1L_c*c_r;
R_r1R_c = F_n_1R_c*c_r;
R_r2L_c = F_n_2L_c*c_r;
R_r2R_c = F_n_2R_c*c_r;

% Longitudinal Force of Each Tire

F_x1L = -R_r1L_c;
F_x1R = -R_r1R_c;
F_x2L = F_tc2L_c-R_r2L_c;
F_x2R = F_tc2R_c-R_r2R_c;

% side slip angle
if u_c == 0
    alpha_f = 0;
    alpha_r = 0;
else
    alpha_f = delta_f_c - (v_c+r_c*l_a)/u_c;
    alpha_r = (r_c*l_b-v_c)/u_c;
end

R_c=l/(delta_f_c-alpha_f+alpha_r);

%
beta = v_c/u_c; % vehicle slip angle (rad)

%
F_y1L = interp2(List_F_ntl,List_a_s,Table_F_lf,F_n_1L_c,alpha_f);
F_y1R = interp2(List_F_ntl,List_a_s,Table_F_lf,F_n_1R_c,alpha_f);
F_y2L = interp2(List_F_ntl,List_a_s,Table_F_lf,F_n_2L_c,alpha_r);
F_y2R = interp2(List_F_ntl,List_a_s,Table_F_lf,F_n_2R_c,alpha_r);

%
du = 1/m_car*[(F_x1L+F_x1R)*cos(delta_f_c)+(F_x2L+F_x2R)-(F_y1L+F_y1R)*sin(delta_f_c)-R_a_c]+r_c*v_c;
dv = 1/m_car*[(F_y2L+F_y2R)+(F_y1L+F_y1R)*cos(delta_f_c)+(F_x1L+F_x1R)*sin(delta_f_c)]-r_c*u_c;
dr = 1/I_z*[l_a*(F_y1L+F_y1R)*cos(delta_f_c)-l_b*(F_y2L+F_y2R)+l_a*(F_x1L+F_x1R)*sin(delta_f_c)+...
    l_d/2*(F_x2R-F_x2L)+l_d/2*(F_x1R-F_x1L)*cos(delta_f_c)-l_d/2*(F_y1R-F_y1L)*sin(delta_f_c)];


u_n = u_c + du*dt;
v_n = v_c + dv*dt;
r_n = r_c + dr*dt;



% Denotation
% u % vehicle longitudinal speed (m/s)
% v % vehicle lateral speed (m/s)
% r % vehicle yaw rate (rad/s)
% du % vehicle longitudinal acceleration (m/s^2)
% dv % vehicle lateral acceleration (m/s^2)
% dr % vehicle yaw rate acceleration (rad/s^2)
% m_car % total weight of vehicle (kg)
% delta_f_c % current front wheel steering angle (rad)
% alpha_f % front wheel side slip angle (rad)
% alpha_r % rear wheel side slip angle (rad)
% R_r1_c % tire 1 rolling resistance (N)
% F_nt1 % tire 1 normal force (N)
% l % wheel base (m)
% l_a % distance between vehicle mass center to front wheel axle (m)
% l_b % distance between vehicle mass center to rear wheel axle (m)
% l_d % width
% beta % vehicle slip angle (rad)

