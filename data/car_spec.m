%
%  Cornering Vehicle Dynamics Simulation
%
%  Created by I.Chen and Y.Huang in 2015.
%  Copyright (c) 2015 All rights reserved.
% 

% =====Vehicle Specification=====
% Porsche GT from Carsim 8
m_car = 1278; % total weight of vehicle (kg)
I_z = 1270; % Yaw inertia (kgm^2)
l = 2.350;% wheel base (m)
l_d = 1.807; % width (m)
l_h = 0.390; % vehicle mass center height (m)
l_a = 1.525; % distance between vehicle mass center to front wheel axle (m)
l_b = l-l_a;% distance between vehicle mass center to rear wheel axle (m)

r_w = 0.292; % wheel radius (m) ****Changed from 0.34
I_w = 2; % inertia of each wheel  (kgm^2)
I_c = 0.1; % inertia of carrier and powertrain (kgm^2)

n_fr = 100; % final reduction gear ratio

A_f = 1.0; % vehicle frontal area
c_d = 0.250; % aerodynamic resistance coefficient
rho_a = 1.208; % air density
c_r = 0.004; % coefficient of rolling resistance
g = 9.81;

% Differential Gear
g_2L = 50;
g_2R = 50;

%Magic Formula Coefficient
a0=1.799;
a1=0;
a2=1688;
a3=4140;
a4=6.026;
a5=0;
a6=-0.3589;
a7=1;
a8=0;
a9=-6.111/1000;
a10=-3.224/100;
a11=0;
a12=0;
a13=0;
a14=0;
a15=0;
a16=0;
a17=0;

b0=1.65;
b1=0;
b2=1688;
b3=0;
b4=229;
b5=0;
b6=0;
b7=0;
b8=-10;
b9=0;
b10=0;
b11=0;
b12=0;
b13=0;

c0=2.0680;
c1=-6.49;
c2=-21.85;
c3=0.4160;
c4=-21.31;
c5=0.02942;
c6=0;
c7=-1.197;
c8=5.228;
c9=-14.84;
c10=0;
c11=0;
c12=-0.003736;
c13=0.03891;
c14=0;
c15=0;
c16=0.6390;
c17=1.693;
c18=0;
c19=0;
c20=0;

a_c=0;%Camber angle asumption

%% the maximum shift speed of CVT ratio (ratio/s)
dn_cvt_max = 0.05; 
