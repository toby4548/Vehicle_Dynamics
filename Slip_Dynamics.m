%
%  Cornering Vehicle Dynamics Simulation
%
%  Created by I.Chen and Y.Huang in 2015.
%  Copyright (c) 2015 All rights reserved.
% 

clc
clear all


% =====System Setting=====
TD_type = 2; %  (1) Locked Axle; (2) Open Differential; (3) DCVT;
cyc_type = 1;
% split road mu
low_mu = 0.01;
high_mu = 0.7;

v_target = 50; % vehicle speed 50

% =====Simulation Setting=====
dt = 0.001; %0.025
slip_time = 0.1;
final_time = 1.5;

% =====Unit Transfer=====
kmh2ms = 1000/60/60; % [km/h]->[m/s]
rpm2rs = 2*pi/60; % [rpm]->[r/s]
deg2rad = pi/180; % [degree]->[rad]

% =====Load Data=====
cd('data'); % path into the folder
car_spec;
magic_formula;
%i_vs_tc;
%a_vs_lf;
cd ..




% =====Initial Condition=====

v_error_p = 0; % speed error
v_error_i = 0;

delta_f_c = 0; % front wheel steering angle
ini_n_CVT = 1;
n_CVT_2L_c = ini_n_CVT ;
n_CVT_2R_c = ini_n_CVT ;
u_c = v_target*kmh2ms; % +:front
v_c = 0; % +:left
r_c = 0; % +:conterclockwise

du = 0;
dv = 0;
dr = 0;
p0_initial % to find initial tire slip
n=1;

for t=0:dt:final_time
    %p1_Driver
    v_error_d = v_error_p-(v_target*kmh2ms-u_c);
v_error_i = v_error_i+v_error_p;%accumulate all the error before
v_error_p = (v_target*kmh2ms-u_c);%find the error between desire speed and true speed

            T_in_c = T_in_c_initial +30000*v_error_p+300*v_error_i+0*v_error_d;% ALL
    %p2_Traction
    r_c=0;
    p2_Traction
    %p3_Vehicle Speed
    du = (T_totaltc_c-R_total_c)/m_car;
    u_n = u_c + du*dt;
    % Record
    sim_time(n) = t; % time
sim_T_in(n) = T_in_c;
sim_v_w2L(n) = v_w2L_c;
sim_v_w2R(n) = v_w2R_c;
sim_F_n_2L(n) = F_n_2L_c;
sim_F_n_2R(n) = F_n_2R_c;
sim_W_w2L(n) = W_w2L_c;
sim_W_w2R(n) = W_w2R_c;
sim_i_s2L(n) = i_s_2L_c;
sim_i_s2R(n) = i_s_2R_c;
sim_slip_w2L(n) = slip_w2L;
sim_slip_w2R(n) = slip_w2R;
sim_F_tc2L(n) = F_tc2L_c;
sim_F_tc2R(n) = F_tc2R_c;
sim_F_totaltc(n) = F_totaltc_c;
sim_T_totaltc(n) = T_totaltc_c;
sim_dW_in(n) = dW_in_c;
sim_du(n)=du;
% sim_R_r2L(n)=R_r2L_c;
% sim_R_r2R(n)=R_r2R_c;
sim_F_n_2L(n)=F_n_2L_c;
sim_F_n_2R(n)=F_n_2R_c;
sim_n_CVT_2L(n) = n_CVT_2L_c;
sim_n_CVT_2R(n) = n_CVT_2R_c;
sim_R_r(n) = R_r_c;
sim_R_a(n) = R_a_c;
sim_R_total(n) = R_total_c;
sim_u(n) = u_c;
sim_r(n) = r_c;


    %VD_next_step
    W_in_c = W_in_n;
    W_w2L_c = W_w2L_n;
    W_w2R_c = W_w2R_n;
    u_c = u_n;
    n = n+1;
    
    
end
VD_save_data
Slip_plot
