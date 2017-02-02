%
%  Cornering Vehicle Dynamics Simulation
%
%  Created by I.Chen and Y.Huang in 2015.
%  Copyright (c) 2015 All rights reserved.
% 


% Traction Effort

% Wheel Moving Speed
v_w2L_c = u_c - l_d/2*r_c; % rear left wheel longitudinal speed (m/s)
v_w2R_c = u_c + l_d/2*r_c; % rear right wheel longitudinal speed (m/s)

% Tire Slip

i_s_2L_c = (r_w*W_w2L_c-v_w2L_c)/abs(r_w*W_w2L_c);
i_s_2R_c = (r_w*W_w2R_c-v_w2R_c)/abs(r_w*W_w2R_c);

%{
if r_w*W_w2R_c >= v_w2R_c
    i_s_2L_c = 1-(v_w2L_c/(r_w*W_w2L_c));
    i_s_2R_c = 1-(v_w2R_c/(r_w*W_w2R_c));
else
    i_s_2L_c = 1-(v_w2L_c/(r_w*W_w2L_c));
    i_s_2R_c = ((r_w*W_w2R_c)/v_w2R_c)-1;
end
%}  
  
% floor slip to prevent numerical error
% i_s_2L_c = floor(i_s_2L_c*1000000)/1000000;
% i_s_2R_c = floor(i_s_2R_c*1000000)/1000000;

% Normal Load of Each Tire
F_n_1L_c = [(1/l*[m_car*g*l_b-l_h*(m_car*du+R_a_c)])*l_d/2-m_car*dv*l_h]/l_d;
F_n_1R_c = [(1/l*[m_car*g*l_b-l_h*(m_car*du+R_a_c)])*l_d/2+m_car*dv*l_h]/l_d;
F_n_2L_c = [(1/l*[m_car*g*l_a+l_h*(m_car*du+R_a_c)])*l_d/2-m_car*dv*l_h]/l_d;
F_n_2R_c = [(1/l*[m_car*g*l_a+l_h*(m_car*du+R_a_c)])*l_d/2+m_car*dv*l_h]/l_d;


% Find Tire Curve
List_F_tc2L_c = (interp2(List_F_nt,List_i_s,Table_F_tc,F_n_2L_c,List_i_s))'; % i_vs_t ccurve for rear left tire
List_F_tc2R_c = (interp2(List_F_nt,List_i_s,Table_F_tc,F_n_2R_c,List_i_s))'; % i_vs_t ccurve for rear right tire

% Interpolate Tractive Effort
F_tc2L_c = sign(i_s_2L_c)*interp1(List_i_s,List_F_tc2L_c,abs(i_s_2L_c)); % tractive effort for rear left tire
F_tc2R_c = sign(i_s_2R_c)*interp1(List_i_s,List_F_tc2R_c,abs(i_s_2R_c)); % tractive effort for rear right tire



% Road Friction
if t>slip_time
    switch cyc_type
        case 1
            if F_tc2L_c>F_n_2L_c*low_mu
                F_tc2L_c=F_n_2L_c*low_mu;
                slip_w2L = 1; % record if tire slip
            else
                slip_w2L = 0;
            end
            if F_tc2R_c>F_n_2R_c*high_mu
                F_tc2R_c=F_n_2R_c*high_mu;
                slip_w2R = 1; % record if tire slip
            else
                slip_w2R = 0;
            end
        case 2
            if F_tc2L_c>F_n_2L_c*high_mu
                F_tc2L_c=F_n_2L_c*high_mu;
                slip_w2L = 1; % record if tire slip
            else
                slip_w2L = 0;
            end
            if F_tc2R_c>F_n_2R_c*high_mu
                F_tc2R_c=F_n_2R_c*high_mu;
                slip_w2R = 1; % record if tire slip
            else
                slip_w2R = 0;
            end
    end
else
    if F_tc2L_c>F_n_2L_c*high_mu
        F_tc2L_c=F_n_2L_c*high_mu;
        slip_w2L = 1; % record if tire slip
    else
        slip_w2L = 0;
    end
    if F_tc2R_c>F_n_2R_c*high_mu
        F_tc2R_c=F_n_2R_c*high_mu;
        slip_w2R = 1; % record if tire slip
    else
        slip_w2R = 0;
    end
end

% Calculate The Total Tration Effort and Torque

switch TD_type
    case 1 % (1) Locked Axle;
    F_totaltc_c = F_tc2L_c+F_tc2R_c; % total tractive effort
    T_totaltc_c = F_totaltc_c*r_w; % total tractive torque
    case 2 % open differential
    F_totaltc_c = F_tc2L_c+F_tc2R_c; % total tractive effort
    T_totaltc_c = F_totaltc_c*r_w; % total tractive torque
    
end
    
% Transmission Device Selection
switch TD_type
    case 1 % (1) Locked Axle;
        % update tire speed
        dW_w2L_c = (T_in_c-T_totaltc_c)/(2*I_w);
        dW_w2R_c = (T_in_c-T_totaltc_c)/(2*I_w);
        dW_in_c = dW_w2L_c;

    case 2 % open differential
        % differential dynamics of open differential
        W2T = [I_w   0      0    -g_2L;% rear left wheel differential gear
            0     I_c       0    (g_2L+g_2R); % carrier gear (c)
            0     0         I_w  -g_2R;  % right wheel shaft 4 gear
            g_2L    -(g_2L+g_2R)  g_2R   0];
        T2W = inv(W2T);
        
        % update tire speed
        dW_w2L_c = [-F_tc2L_c*r_w*T2W(1,1)+T_in_c*T2W(1,2)-F_tc2R_c*r_w*T2W(1,3)]; % acceleration of shaft 3 (rad/s^2)
        dW_w2R_c = [-F_tc2L_c*r_w*T2W(3,1)+T_in_c*T2W(3,2)-F_tc2R_c*r_w*T2W(3,3)]; % acceleration of shaft 4 (rad/s^2)
        dW_ca_c = [-F_tc2L_c*r_w*T2W(2,1)+T_in_c*T2W(2,2)-F_tc2R_c*r_w*T2W(2,3)]; % acceleration of carrier (rad/s^2)
        dW_in_c = dW_ca_c;
           
        
end

% Calculate Next Wheel Speed
switch TD_type
    case 1
        W_in_n = W_in_c + dW_in_c*dt;
        W_w2L_n = W_w2L_c + dW_w2L_c*dt;
        W_w2R_n = W_w2R_c + dW_w2R_c*dt;
    case 2
        W_in_n = W_in_c + dW_in_c*dt;
        W_w2L_n = W_w2L_c + dW_w2L_c*dt;
        W_w2R_n = W_w2R_c + dW_w2R_c*dt;
    case 3
        W_in_n = W_in_c + dW_in_c*dt;
        W_w2L_n =  W_in_n /n_CVT_2L_n;
        W_w2R_n =  W_in_n /n_CVT_2R_n;
end
% Resistance
R_r_c = m_car*g*c_r;
R_a_c = 1/2*c_d*rho_a*A_f*u_c^2;
R_total_c = R_r_c + R_a_c;


