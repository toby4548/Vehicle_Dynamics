%
%  Cornering Vehicle Dynamics Simulation
%
%  Created by I.Chen and Y.Huang in 2015.
%  Copyright (c) 2015 All rights reserved.
% 


clear all
clc

% =====Simulation Setting=====
dt = 0.001; %0.025
slip_time = 0.1;
final_time = 1.5;
n=1;
sim_time(n)=0;

I_w=2;
I_c=0.1;
g_2L = 1;
g_2R = 1;
W_w2L_SA_c=10;
W_w2R_SA_c=10;
W_in_SA_c = 10;

W_w2L_OD_c=10;
W_w2R_OD_c=10;
W_in_OD_c = 10;


r_w=0.292;
TD_type=1;


for t=0:dt:final_time
    
    
          if t<slip_time
            T_in_c=200;
            F_tc2L_c=100/r_w;
            F_tc2R_c=100/r_w;
            
            T_in_SA_c=200;
            F_tc2L_SA_c=100/r_w;
            F_tc2R_SA_c=100/r_w;
            
            T_in_OD_c=200;
            F_tc2L_OD_c=100/r_w;
            F_tc2R_OD_c=100/r_w;
        else 
            F_tc2L_c=20/r_w;
            T_in_c=200;
            F_tc2R_c=100/r_w;
            
            T_in_SA_c=200;
            F_tc2L_SA_c=20/r_w;
            F_tc2R_SA_c=100/r_w;
            
            T_in_OD_c=200;
            F_tc2L_OD_c=20/r_w;
            F_tc2R_OD_c=100/r_w;
        end
    
%     switch TD_type
%     case 1 % (1) Solid Axle;
    F_totaltc_SA_c = F_tc2L_SA_c+F_tc2R_SA_c; % total tractive effort
    T_totaltc_SA_c = F_totaltc_SA_c*r_w; % total tractive torque
%     case 2 % open differential
    F_totaltc_OD_c = F_tc2L_OD_c+F_tc2R_OD_c; % total tractive effort
    T_totaltc_OD_c = F_totaltc_OD_c*r_w; % total tractive torque
%     case 3 % (3) DCVT;
        
%     F_totaltc_DCVT_c = F_tc2L_c/n_CVT_2L_c+F_tc2R_c/n_CVT_2R_c; %tractive effort ratio
%     T_totaltc_DCVT_c = F_totaltc_c*r_w; % total tractive torque ratio
    %F_totaltc_c = F_tc2L_c+F_tc2R_c; % total tractive effort
    %T_totaltc_c = F_totaltc_c*r_w; % total tractive torque
    
% end

    
    
%     switch TD_type
%     case 1 % (1) Locked Axle;
        % update tire speed
        dW_w2L_SA_c = (T_in_SA_c-T_totaltc_SA_c)/(2*I_w+I_c);
        dW_w2R_SA_c = (T_in_SA_c-T_totaltc_SA_c)/(2*I_w+I_c);
        dW_in_SA_c = dW_w2L_SA_c;

%     case 2
 % differential dynamics of open differential
        W2T = [I_w   0      0    -g_2L;% rear left wheel differential gear
            0     I_c       0    (g_2L+g_2R); % carrier gear (c)
            0     0         I_w  -g_2R;  % right wheel shaft 4 gear
            g_2L    -(g_2L+g_2R)  g_2R   0];
        T2W = inv(W2T);
%     end
        
        % update tire speed

  
  
%         switch TD_type
%         case 1
        W_in_SA_n = W_in_SA_c + dW_in_SA_c*dt;
        W_w2L_SA_n = W_w2L_SA_c + dW_w2L_SA_c*dt;
        W_w2R_SA_n = W_w2R_SA_c + dW_w2R_SA_c*dt;
%         case 2
        dW_w2L_OD_c = [-F_tc2L_OD_c*r_w*T2W(1,1)+T_in_OD_c*T2W(1,2)-F_tc2R_OD_c*r_w*T2W(1,3)]; % acceleration of shaft 3 (rad/s^2)
        dW_w2R_OD_c = [-F_tc2L_OD_c*r_w*T2W(3,1)+T_in_OD_c*T2W(3,2)-F_tc2R_OD_c*r_w*T2W(3,3)]; % acceleration of shaft 4 (rad/s^2)
        dW_ca_OD_c = [-F_tc2L_OD_c*r_w*T2W(2,1)+T_in_OD_c*T2W(2,2)-F_tc2R_OD_c*r_w*T2W(2,3)]; % acceleration of carrier (rad/s^2)
        dW_in_OD_c = dW_ca_OD_c;
        
        
        W_in_OD_n = W_in_OD_c + dW_in_OD_c*dt;
        W_w2L_OD_n = W_w2L_OD_c + dW_w2L_OD_c*dt;
        W_w2R_OD_n = W_w2R_OD_c + dW_w2R_OD_c*dt;
%         end
        
        W_w2L_SA_c=W_w2L_SA_n;
        W_w2R_SA_c=W_w2R_SA_n;
        W_in_SA_c=W_in_SA_n;

        W_w2L_OD_c=W_w2L_OD_n;
        W_w2R_OD_c=W_w2R_OD_n;
        W_in_OD_c=W_in_OD_n;
        
        
        sim_W_w2L_SA(n)=W_w2L_SA_c;
        sim_W_w2R_SA(n)=W_w2R_SA_c;
        sim_W_in_SA(n)=W_in_SA_c;
        sim_T_in_SA(n)=T_in_SA_c;
        sim_T_tc2L_SA(n)=F_tc2L_SA_c*r_w;
        sim_T_tc2R_SA(n)=F_tc2R_SA_c*r_w;
                
        sim_W_w2L_OD(n)=W_w2L_OD_c;
        sim_W_w2R_OD(n)=W_w2R_OD_c;
        sim_W_in_OD(n)=W_in_OD_c;
        sim_T_in_OD(n)=T_in_OD_c;
        sim_T_tc2L_OD(n)=F_tc2L_OD_c*r_w;
        sim_T_tc2R_OD(n)=F_tc2R_OD_c*r_w;
        sim_time(n)=t;
        
        sim_time(n)=t;
        n=n+1;
         
         
        
end     

hold all

%%%%%
figure(1)
set(gcf,'Units','centimeters','position',[15 6 10 5]);
% plot(sim_time,sim_T_in_SA,'LineWidth',2,'Color',[0 0 1]);
hold on

plot(sim_time,sim_T_tc2L_SA,'-.','color', [1 0.5 0],'LineWidth',2);
plot(sim_time,sim_T_tc2R_SA,'-','color', [1 0.5 0],'LineWidth',2);

plot(sim_time,sim_T_tc2L_OD,'b-.','linewidth', 1.5);
plot(sim_time,sim_T_tc2R_OD,'b-','linewidth', 1.5);

ylim([0 120]);
xlabel('Time(s)');
ylabel('Torque(Nm)');
grid on

%%%%%

figure(2)
set(gcf,'Units','centimeters','position',[27 6 10 9]);

plot(sim_time,sim_W_in_SA*180/pi,'LineWidth',2,'Color',[0 0 1]);
hold on

plot(sim_time,sim_W_w2L_SA*180/pi,'-.','color', [1 0.5 0],'LineWidth',2);
plot(sim_time,sim_W_w2R_SA*180/pi,'-', 'color', [1 0.5 0],'linewidth', 2);

plot(sim_time,sim_W_w2L_OD*180/pi,'b-.','LineWidth',1.5);
plot(sim_time,sim_W_w2R_OD*180/pi,'b-','linewidth', 1.5);


xlabel('Time(s)');
ylabel('Rotating Speed(rpm)');
grid on


