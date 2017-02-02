%
%  Cornering Vehicle Dynamics Simulation
%
%  Created by I.Chen and Y.Huang in 2015.
%  Copyright (c) 2015 All rights reserved.
% 

switch TD_type
    case 1 % (1) Locked Axle;
        switch cyc_type
            case 1
                savename=(['Split_mu_LA_Speed_',num2str(v_target),'_ftime_',num2str(final_time),'.mat']);
                save(savename,'sim_time','sim_W_w2L','sim_W_w2R','sim_i_s2L','sim_i_s2R','sim_F_tc2L','sim_F_tc2R','sim_F_totaltc','sim_u','sim_r','sim_T_in');
            case 2
                savename=(['Turning_LA_Speed_',num2str(v_target),'_Steer_',num2str(delta_f_ctrl),'_ftime_',num2str(final_time),'.mat']);
                save(savename,'sim_time','sim_W_w2L','sim_W_w2R','sim_i_s2L','sim_i_s2R','sim_F_tc2L','sim_F_tc2R','sim_F_totaltc','sim_u','sim_v','sim_r','sim_x','sim_y','sim_R','sim_T_in');
        end
    case 2 % Open Differential
        switch cyc_type
            case 1
                savename=(['Split_mu_OD_Speed_',num2str(v_target),'_ftime_',num2str(final_time),'.mat']);
                save(savename,'sim_time','sim_W_w2L','sim_W_w2R','sim_i_s2L','sim_i_s2R','sim_F_tc2L','sim_F_tc2R','sim_F_totaltc','sim_u','sim_r','sim_T_in','sim_T_in');
            case 2
                savename=(['Turning_OD_Speed_',num2str(v_target),'_Steer_',num2str(delta_f_ctrl),'_ftime_',num2str(final_time),'.mat']);
                save(savename,'sim_time','sim_W_w2L','sim_W_w2R','sim_i_s2L','sim_i_s2R','sim_F_tc2L','sim_F_tc2R','sim_F_totaltc','sim_u','sim_v','sim_r','sim_x','sim_y','sim_R','sim_T_in');
        end
    case 3 % (3) DCVT;
        switch cyc_type
            case 1
                savename=(['Split_mu_DCVT_',num2str(DCVT_type),'_Speed_',num2str(v_target),'_ftime_',num2str(final_time),'.mat']);
                save(savename,'sim_time','sim_W_w2L','sim_W_w2R','sim_i_s2L','sim_i_s2R','sim_F_tc2L','sim_F_tc2R','sim_F_totaltc','sim_u','sim_v','sim_r','sim_T_in');
            case 2
                savename=(['Turning_DCVT_',num2str(DCVT_type),'_Speed_',num2str(v_target),'_Steer_',num2str(delta_f_ctrl),'_Gain_',num2str(dcvt_gain),'_Shift_',num2str(dn_cvt_max),'_ftime_',num2str(final_time),'.mat']);
                save(savename,'sim_time','sim_W_w2L','sim_W_w2R','sim_i_s2L','sim_i_s2R','sim_F_tc2L','sim_F_tc2R','sim_F_totaltc','sim_u','sim_v','sim_r','sim_x','sim_y','sim_R','sim_T_in');
        end
end

% loadname=(['Split_mu_LA_Speed',num2str(v_target),'.mat']);
% load(loadname,'sim_time','sim_W_w2L','sim_i_s2L','sim_i_s2R','sim_F_tc2L','sim_F_tc2R','sim_F_totaltc','sim_u','sim_v','sim_r');
% loadname=(['Turning_LA_Speed_',num2str(v_target),'_Steer_',num2str(delta_f_ctrl),'.mat']);
% load(loadname,'sim_time','sim_W_w2L','sim_i_s2L','sim_i_s2R','sim_F_tc2L','sim_F_tc2R','sim_F_totaltc','sim_u','sim_v','sim_r');
