
%%%%%%
figure(1)
set(gcf,'Units','centimeters','position',[3 5 25 10]);

subplot(2,3,1);
plot(sim_time, sim_W_w2L/rpm2rs, 'color', [0 0.7 0],'linewidth',2);
ylabel({'Wheel speed', '(rpm)'});
xlabel({'Time(s)'});
grid on
hold on
plot(sim_time, sim_W_w2R/rpm2rs,'color', [1 0 0],'linewidth',2);
grid on

subplot(2,3,2);
plot(sim_time, sim_T_in,'color', [1 0.5 0],'linewidth',2);
ylabel({'T in (Nm)'});
xlabel({'Time (s)'});
grid on
hold on

subplot(2,3,3);
plot(sim_time, sim_u*3.6,'color', [1 0.5 0],'linewidth',2);
ylabel({'u', '(km/h)'});
xlabel({'Time(s)'});
grid on
hold on

subplot(2,3,4);
plot(sim_time, sim_i_s2L, 'color', [0 0.7 0],'linewidth',2);
grid on
hold on
plot(sim_time, sim_i_s2R,'color', [1 0 0],'linewidth',2);
ylabel({'Tire', 'slip ratio'});
xlabel({'Time(s)'});
grid on

subplot(2,3,5);
plot(sim_time, sim_F_tc2L, 'color', [0 0.7 0],'linewidth',2);
grid on
hold on
plot(sim_time, sim_F_tc2R, 'color', [1 0 0],'linewidth',2);
ylabel({'Tire traction', '(N)'});
xlabel({'Time(s)'});
grid on

subplot(2,3,6);
plot(sim_time, sim_F_totaltc, 'color', [0 0.7 0],'linewidth',2);
grid on
hold on
ylabel({'Total Traction', '(N)'});
xlabel({'Time(s)'});
grid on









