%
%  Cornering Vehicle Dynamics Simulation
%
%  Created by I.Chen and Y.Huang in 2015.
%  Copyright (c) 2015 All rights reserved.
% 


% Calculate Resistance
R_r_c = m_car*g*c_r;
R_a_c = 1/2*c_d*rho_a*A_f*u_c^2; 
R_total_c = R_r_c + R_a_c;

% Normal Load of Each Tire
F_n_1L_c = [(1/l*[m_car*g*l_b-l_h*(m_car*du+R_a_c)])*l_d/2-m_car*dv*l_h]/l_d;
F_n_1R_c = [(1/l*[m_car*g*l_b-l_h*(m_car*du+R_a_c)])*l_d/2+m_car*dv*l_h]/l_d; 
F_n_2L_c = [(1/l*[m_car*g*l_a+l_h*(m_car*du+R_a_c)])*l_d/2-m_car*dv*l_h]/l_d; 
F_n_2R_c = [(1/l*[m_car*g*l_a+l_h*(m_car*du+R_a_c)])*l_d/2+m_car*dv*l_h]/l_d; 

% Build Tire Slip vs Traction Table for each tire
List_itc_2L_c = (interp2(List_F_nt,List_i_s,Table_F_tc,F_n_2L_c,List_i_s))';
List_itc_2R_c = (interp2(List_F_nt,List_i_s,Table_F_tc,F_n_2R_c,List_i_s))'; 

% Interpolate Resistance(Traction) to Find Tire Slip Ratio
i_s_2L_c = interp1(List_itc_2L_c(1:7),List_i_s(1:7),R_total_c/2);
i_s_2R_c = interp1(List_itc_2R_c(1:7),List_i_s(1:7),R_total_c/2);

% Calculate Traction
F_tc_2L_c = interp1(List_i_s(1:7),List_itc_2L_c(1:7),i_s_2L_c); 
F_tc_2R_c = interp1(List_i_s(1:7),List_itc_2R_c(1:7),i_s_2R_c); 



% Calculate Wheel Speed
W_w2L_c =  (u_c/r_w)/(1-i_s_2L_c);
W_w2R_c =  (u_c/r_w)/(1-i_s_2R_c);

% Calculate Input Speed
switch TD_type
    case 1 
        W_in_c = W_w2L_c;
    case 2
        W_in_c = (W_w2L_c+W_w2R_c)/2;
    case 3
        W_in_c = W_w2L_c*n_CVT_2L_c;
end

% Calculate Total Traction Force and Torque
F_totaltc_c = F_tc_2L_c+F_tc_2R_c; % total tractive effort
T_totaltc_c = F_totaltc_c*r_w; % total tractive torque

% System input Torque
T_in_c = T_totaltc_c;
T_in_c_initial=T_in_c;
