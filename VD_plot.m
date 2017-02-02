%
%  Cornering Vehicle Dynamics Simulation
%
%  Created by I.Chen and Y.Huang in 2015.
%  Copyright (c) 2015 All rights reserved.
% 


figure(1)
set(gcf,'Units','centimeters','position',[3 2 30 15]);

%%  Wheel Speed
subplot(3,3,1);
%figure(1)
%set(gcf,'Units','centimeters','position',[3 2 20 7.5]);
plot(sim_time, sim_W_w2L/rpm2rs, 'color', [0 0.7 0],'linewidth',2);
ylabel({'Wheel speed', '(rpm)'});
xlabel({'Time(s)'});
hold on
plot(sim_time, sim_W_w2R/rpm2rs,'--','color', [1 0 0],'linewidth',2);
grid on

%plot(Carsim_SA_Wheel_Speed(:,1),Carsim_SA_Wheel_Speed(:,2),'--', 'color', [0 0.7 0],'linewidth',2);
%plot(Carsim_SA_Wheel_Speed(:,1),Carsim_SA_Wheel_Speed(:,3),'--','color', [1 0 0],'linewidth',2);

%% Slip

subplot(3,3,2);
%figure(2)
%set(gcf,'Units','centimeters','position',[3 2 20 7.5]);
plot(sim_time, sim_i_s2L, 'color', [0 0.7 0],'linewidth',2);
hold on
plot(sim_time, sim_i_s2R,'--','color', [1 0 0],'linewidth',2);
ylabel({'Tire', 'slip ratio'});
xlabel({'Time(s)'});
grid on
%plot(Carsim_SA_Longitudinal_Slip(:,1),Carsim_SA_Longitudinal_Slip(:,2),'--', 'color', [0 0.7 0],'linewidth',2);
%plot(Carsim_SA_Longitudinal_Slip(:,1),Carsim_SA_Longitudinal_Slip(:,3),'--','color', [1 0 0],'linewidth',2);

%% Traction Force

subplot(3,3,3);
%figure(3)
%set(gcf,'Units','centimeters','position',[3 2 20 7.5]);
plot(sim_time, sim_F_tc2L, 'color', [0 0.7 0],'linewidth',2);
hold on
plot(sim_time, sim_F_tc2R,'--', 'color', [1 0 0],'linewidth',2);
hold on
plot(sim_time, sim_F_totaltc,':', 'color', [0 0.5 1],'linewidth',2);
ylabel({'Tire traction', '(N)'});
xlabel({'Time(s)'});
grid on
%plot(Carsim_SA_Longitudinal_Force(:,1), Carsim_SA_Longitudinal_Force(:,2),'--', 'color', [0 0.7 0],'linewidth',2);
%plot(Carsim_SA_Longitudinal_Force(:,1), Carsim_SA_Longitudinal_Force(:,3),'--', 'color', [1 0 0],'linewidth',2);
%plot(Carsim_SA_Longitudinal_Force(:,1), Carsim_SA_Longitudinal_Force(:,4),'--', 'color', [0 0.5 1],'linewidth',2);

%% Tire Load

subplot(3,3,4);
%figure(4)
%set(gcf,'Units','centimeters','position',[3 2 20 7.5]);
plot(sim_time, sim_F_n_2L, 'color', [0 0.7 0],'linewidth',2);
hold on
plot(sim_time, sim_F_n_2R,'--', 'color', [1 0 0],'linewidth',2);
ylabel({'Tire Load', '(N)'});
xlabel({'Time(s)'});
grid on

%% Position

subplot(3,3,6);
%figure(6)
%set(gcf,'Units','centimeters','position',[3 2 20 7.5]);
plot(sim_x, sim_y,'color', [1 0.5 0],'linewidth',2);
ylabel({'y', '(m)'});
xlabel({'x', '(m)'});
hold on
grid on
%load('C:\Users\Tony\Dropbox\DCVT\Master Thesis\Program\2015_06_07 DCVT VD\OD M1\Carsim_OD_Longitudinal_Speed.mat');
%plot(Carsim_OD_Longitudinal_Speed(:,1),Carsim_OD_Longitudinal_Speed(:,2),'--','color', [1 0.5 0],'linewidth',2);

%% u

subplot(3,3,7)
%figure(7)
%set(gcf,'Units','centimeters','position',[3 2 20 7.5]);
plot(sim_time, sim_u*3.6,'color', [1 0.5 0],'linewidth',2);
ylabel({'u', '(km/h)'});
xlabel({'Time(s)'});
grid on
hold on
%plot(Carsim_OD_Longitudinal_Speed(:,1),Carsim_OD_Longitudinal_Speed(:,2),'+-','color', [1 0.5 0],'linewidth',2);


%% v

subplot(3,3,8);
%figure(8)
%set(gcf,'Units','centimeters','position',[3 2 20 7.5]);
plot(sim_time, sim_v, 'color', [1 0.5 0],'linewidth',2);
ylabel({'v', '(m/s)'});
xlabel({'Time(s)'});
grid on

%% r

subplot(3,3,9);
%figure(9)
%set(gcf,'Units','centimeters','position',[3 2 20 7.5]);
plot(sim_time, sim_r, 'color',[1 0.5 0],'linewidth',2);
ylabel({'r', '(rad/s)'});
xlabel({'Time(s)'});
grid on

%h2=figure(2);
h2=figure(1);


% =====Save Plot=====
switch TD_type
    case 1 % (1) Locked Axle;
        switch cyc_type
            case 1
%                 saveas( h1 , ['Split_mu_plot1_LA_Speed_',num2str(v_target),'.tiff']);
                saveas( h2 , ['Split_mu_plot2_Speed_',num2str(v_target),'_LA.tiff']);
            case 2
%                 saveas( h1 , ['Turning_plot1_LA_Speed_',num2str(v_target),'_Steer_',num2str(delta_f_ctrl),'.tiff']);
                saveas( h2 , ['Turning_plot2_Speed_',num2str(v_target),'_Steer_',num2str(delta_f_ctrl),'_LA.tiff']);
        end
    case 2 % Open Differential
        switch cyc_type
            case 1
%                 saveas( h1 , ['Split_mu_plot1_OD_Speed_',num2str(v_target),'.tiff']);
                saveas( h2 , ['Split_mu_plot2_Speed_',num2str(v_target),'_OD.tiff']);
            case 2
%                 saveas( h1 , ['Turning_plot1_OD_Speed_',num2str(v_target),'_Steer_',num2str(delta_f_ctrl),'.tiff']);
                saveas( h2 , ['Turning_plot2_Speed_',num2str(v_target),'_Steer_',num2str(delta_f_ctrl),'_OD.tiff']);
        end
    case 3 % (3) DCVT;
        switch cyc_type
            case 1
%                 saveas( h1 , ['Split_mu_plot1_DCVT_Speed_',num2str(v_target),'.tiff']);
                saveas( h2 , ['Split_mu_plot2_Speed_',num2str(v_target),'_DCVT_',num2str(DCVT_type),'.tiff']);
            case 2
%                 saveas( h1 , ['Turning_plot1_DCVT_Speed_',num2str(v_target),'_Steer_',num2str(delta_f_ctrl),'.tiff']);
                saveas( h2 , ['Turning_plot2_Speed_',num2str(v_target),'_Steer_',num2str(delta_f_ctrl),'_DCVT_',num2str(DCVT_type),'.tiff']);
        end
end
