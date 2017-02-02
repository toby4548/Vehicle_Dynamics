%
%  Cornering Vehicle Dynamics Simulation
%
%  Created by I.Chen and Y.Huang in 2015.
%  Copyright (c) 2015 All rights reserved.
% 



clc
clear all

% =====System Setting=====
TD_type = 2; %  (1) Locked Axle; (2) Open Differential;
cyc_type = 2; % (1) split road mu; (2) constant cornering
DCVT_type = 2; % (1) under steering (2) neutral steering (3) over steering

% split road mu
low_mu = 0.011;
high_mu = 0.7;

% constant cornering
v_target = 50; % vehicle speed 60
R_target = 25;

delta_f_ctrl = 8; % designed frontal steering angle (deg) max20

%{
%% DCVT setting
% the maximum shift speed of CVT ratio (ratio/s)
dn_cvt_max = 0.05; 
% DCVT gain
switch DCVT_type
    case 1 % (1) under steering
        dcvt_gain = 0.1;
    case 2 %(2) neutral steering
        dcvt_gain = 0.3975;
        %dcvt_gain=0.4;
    case 3 %(3) over steering
        dcvt_gain = 0.7;
end
%}


% =====Simulation Setting=====
dt = 0.001; %0.025
slip_time = 0.1;
turning_time = 0.1;
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
switch cyc_type
    case 1 % (1) split road mu
        
        x_c=0; %vehicle position x
        y_c=0; %vehicle position y
        theta_c=0;
        slip_w2L=0;
        slip_w2R=0;
        
        v_error_p = 0; % speed error
        v_error_i = 0;
        delta_f_c = 0; % front wheel steering angle
        ini_n_CVT = 1;
        n_CVT_2L_c = ini_n_CVT ;
        n_CVT_2R_c = ini_n_CVT ;
        %u_c = 0.1; % +:front
        u_c = v_target*kmh2ms;
        v_c = 0; % +:left
        r_c = 0; % +:conterclockwise
        du = 0;
        dv = 0;
        dr = 0;
        p0_initial % to find initial tire slip
        n=1;
    case 2 % (2) constant cornering
        
        x_c=0; %vehicle position x
        y_c=0; %vehicle position y
        theta_c=0;
        
        v_error_p = 0; % speed error
        v_error_i = 0;
        R_error_p = 0;
        R_error_i = 0;
        delta_f_c = 0; % front wheel steering angle
        ini_n_CVT = 1;
        n_CVT_2L_c = ini_n_CVT ;
        n_CVT_2R_c = ini_n_CVT ;
        u_c = v_target*kmh2ms; % +:front
        v_c = 0; % +:left
        r_c = 0; % +:conterclockwise
        R_c = 25;
        du = 0;
        dv = 0;
        dr = 0;
        p0_initial % to find initial tire slip
        n=1;
end

% =====Simulation Process=====

for t=0:dt:final_time
    p1_Driver
    p2_Traction
    p3_VD
    p4_Trajectory
    VD_sim_record
    VD_next_step
end

% =====Data Output=====
VD_save_data
VD_plot

% ====Turning Radius=====

 x1=sim_x(n-3);y1=sim_y(n-3);% use the latest 3 position of the vehicle to find a circle 
 x2=sim_x(n-2);y2=sim_y(n-2);
 x3=sim_x(n-1);y3=sim_y(n-1);
 a=2*(x2-x1);
 b=2*(y2-y1);
 c=x2*x2+y2*y2-x1*x1-y1*y1;
 d=2*(x3-x2);
 e=2*(y3-y2);
 f=x3*x3+y3*y3-x2*x2-y2*y2;
 %find center
 x0=(b*f-e*c)/(b*d-e*a);
 y0=(d*c-a*f)/(b*d-e*a);
 %find turning radius
 %disp('Turning Radius(m)::');
 r=sqrt((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1));%true
 Rg=l/(delta_f_c-alpha_f+alpha_r);%Estimate

  
 %Steady State Turning Circle Plot
 theta = linspace(0, 2*pi); 
 rho = r; 
 [x, y] = pol2cart(theta, rho); 
 subplot(3,3,6);
 plot(x+x0, y+y0,'--k');
 
 


 