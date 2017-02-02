%
%  Cornering Vehicle Dynamics Simulation
%
%  Created by I.Chen and Y.Huang in 2015.
%  Copyright (c) 2015 All rights reserved.
% 


%MF.m Magic Formula. To replace the file a_vs_lf.m and i_vs_tc.m  to
%calculate traction force lateral force and aligning moment. The formula is
%from the book- Motor vehicle dynamics by Giancarlo Genta

%%
%Traction Force
List_F_nt = [0 2105 3995 6120 7900 10100]/1000; % list of normal force on tire (kN)
[i,j]=size(List_F_nt); 

List_i_s = [0 0.02 0.04 0.06 0.08 0.1 0.12 0.14 0.16 0.18 0.2 0.25 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0]; % list of tire slip ratio
[m,n]=size(List_i_s);

for Row_x=1:j
    for Col_x=1:n

Sh_x=b9*List_F_nt(1,Row_x)+b10;
Sv_x=b11*List_F_nt(1,Row_x)+b12;
C_x=b0;
D_x=List_F_nt(1,Row_x)*(b1*List_F_nt(1,Row_x)+b2);
B_x=(((b3*List_F_nt(1,Row_x)^2)+b4*List_F_nt(1,Row_x))*exp(-b5*List_F_nt(1,Row_x)))/(C_x*D_x);
E_x=((b6*List_F_nt(1,Row_x)^2)+b7*List_F_nt(1,Row_x)+b8)*(1-b13*sign(1));%E_x=((b6*List_F_nt(1,Row_x)^2)+b7*List_F_nt(1,Row_x)+b8)*(1-b13*sign(List_i_s(i,1)+Sh_x))

Table_F_tc(Col_x,Row_x)=D_x*sin(C_x*atan(B_x*(1-E_x)*(100*List_i_s(1,Col_x)+Sh_x)+E_x*atan(B_x*(100*List_i_s(1,Col_x)+Sh_x))))+Sv_x;

    end
end


%%
%Lateral Force
List_F_ntl = [0	1725 3500 6100 6950 9005]/1000; % list of normal force on tire (kN)
[o,p]=size(List_F_ntl);
List_a_s = [0 0.5 1 1.5 2 2.5 3 3.5 4 4.5 5 5.5 6 6.5 7 7.5 8 8.5 9 9.5 10 12 14 16 18 20]; % list of tire slip angle(deg)
[q,r]=size(List_a_s);

for Row_y=1:p
    for Col_y=1:r

Sh_y=a8*List_F_ntl(1,Row_y)+a9+a10*a_c;
Sv_y=a11*List_F_ntl(1,Row_y)+a12+(a13*List_F_ntl(1,Row_y)*List_F_ntl(1,Row_y)+a14*List_F_ntl(1,Row_y))*a_c;
C_y=a0;
D_y=List_F_ntl(1,Row_y)*(a1*List_F_ntl(1,Row_y)+a2)*(1-a15*a_c*a_c);
B_y=(a3*sin(2*atan(List_F_ntl(1,Row_y)/a4))*(1-a5*abs(a_c)))/(C_y*D_y);
E_y=(a6*List_F_ntl(1,Row_y)+a7)*(1-(a16*a_c+a17)*sign(1));%E_y=(a6*List_F_ntl(1,Row_y)+a7)(1-(a16*a_c+a17)*sign(List_a_s(i,1+Sh_y)))

Table_F_lf(Col_y,Row_y)=D_y*sin(C_y*atan(B_y*(1-E_y)*(List_a_s(1,Col_y)+Sh_y)+E_y*atan(B_y*(List_a_s(1,Col_y)+Sh_y))))+Sv_y;

 end
end

%%
%Aligning Moment
List_F_ntm = [0 1715.32 3430.65 5145.97 6861.29 8576.62]/1000; % list of normal force on tire (kN)
[w,x]=size(List_F_ntm);

List_a_s_m = [0 0.5 1 1.5 2 2.5 3 3.5 4 4.5 5 5.5 6 6.5 7 7.5 8 8.5 9 9.5 10 11]; % list of tire slip angle(deg)
[y,z]=size(List_a_s_m);

for Row_z=1:x
    for Col_z=1:z

Sh_z=c11*a_c+c12*List_F_ntm(1,Row_z)+c13;
Sv_z=(c16*List_F_ntm(1,Row_z)^2+c17*List_F_ntm(1,Row_z))*a_c+c14*List_F_ntm(1,Row_z)+c15;
C_z=c0;
D_z=c1*List_F_ntm(1,Row_z)^2+c2*List_F_ntm(1,Row_z)*(1-c18*a_c^2);
B_z=((c3*List_F_ntm(1,Row_z)^2+c4*List_F_ntm(1,Row_z))*(1-c6*abs(a_c))*exp(-c5*List_F_ntm(1,Row_z)))/(C_z*D_z);
E_z=(c7*List_F_ntm(1,Row_z)^2+c8*List_F_ntm(1,Row_z)+c9)*(1-(c19*a_c+c20)*sign(1))/(1-c10*abs(a_c));%E_z=(c7*List_F_ntm(1,Row_z)^2+c8*List_F_ntm(1,Row_z)+c9)*(1-(c19*a_c+c20)*sign(List_a_s_m(i,1)+Sh_z))/(1-c10*abs(a_c))

Table_M_a(Col_z,Row_z)=D_z*sin(C_z*atan(B_z*(1-E_z)*(List_a_s_m(1,Col_z)+Sh_z)+E_z*atan(B_z*(List_a_s_m(1,Col_z)+Sh_z))))+Sv_z;
%Table_M_a(Col_z,Row_z)=-1*(D_z*sin(C_z*atan(B_z*(1-E_z)*(List_a_s_m(1,Col_z)+Sh_z)+E_z*atan(B_z*(List_a_s_m(1,Col_z)+Sh_z))))+Sv_z);

    end
end

%%
%Change Data Format
Table_F_tc(:,1)=0;
Table_F_lf(:,1)=0;
Table_F_lf(1,:)=0;
Table_M_a(:,1)=0;
Table_M_a(1,:)=0;

List_F_nt=List_F_nt*1000;
List_F_ntl=List_F_ntl*1000;
List_F_ntm=List_F_ntm*1000;

List_a_s=List_a_s*deg2rad;
List_a_s_m=List_a_s_m*deg2rad;
