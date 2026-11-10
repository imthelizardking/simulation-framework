classdef AeroCompound6DoF < BaseAero
    properties
        geom
        aero_tables
        downwash_grad = 0.15;
        HingeMoments = zeros(12,1);
    end

    methods
        function obj = AeroCompound6DoF(geom_params, tables)
            obj.geom = geom_params;
            obj.aero_tables = tables;
        end

        % BaseAero standart imza
        function [F_aero, M_aero] = getForcesAndMoments(obj, t, X, env, fin_deflections_cmd)
            u = X(4); v = X(5); w = X(6);
            p = X(10); q = X(11); r = X(12);

            % Framework'ün standart akış (flow) hesaplayıcısını kullanıyoruz
            flow = BaseAero.computeFlowState(X, env, obj.Frame);
            Mach = flow.mach;
            qbar = flow.q_bar;

            % DİKKAT: Komut edilen değil, RK4'ten gelen fiziksel X state'ini okuyoruz
            if length(X) >= 33
                servos_state = X(22:33);
            else
                servos_state = zeros(12,1);
            end

            V_total = sqrt(u^2 + v^2 + w^2);
            if V_total < 1e-3; V_total = 1e-3; end

            xc = obj.geom.x_canard_AC - obj.geom.x_CG;
            xt = obj.geom.x_tail_AC - obj.geom.x_CG;
            R = obj.geom.body_radius;

            y_pos = [0; R; 0; -R];
            z_pos = [-R; 0; R; 0];

            alpha_loc_c = zeros(4,1);
            alpha_loc_t = zeros(4,1);

            for i = 1:4
                v_loc_c = v + p*z_pos(i) - r*xc;
                w_loc_c = w - p*y_pos(i) + q*xc;
                v_loc_t = v + p*z_pos(i) - r*xt;
                w_loc_t = w - p*y_pos(i) + q*xt;

                if i==1 || i==3
                    alpha_loc_c(i) = atan2(v_loc_c, u) + servos_state(i);
                    alpha_loc_t(i) = atan2(v_loc_t, u) + servos_state(i+4);
                else
                    alpha_loc_c(i) = atan2(w_loc_c, u) + servos_state(i);
                    alpha_loc_t(i) = atan2(w_loc_t, u) + servos_state(i+4);
                end
            end

            % Downwash & Interference (Mock Values)
            alpha_eff_c = alpha_loc_c;
            alpha_eff_t = alpha_loc_t - obj.downwash_grad * alpha_eff_c;
            K_WB = 1.2; K_BW = 0.3;

            F_N_c = zeros(4,1); HM_c = zeros(4,1);
            F_N_t = zeros(4,1); HM_t = zeros(4,1);

            for i=1:4
                CN_c = 0.1 * alpha_eff_c(i);
                F_N_c(i) = qbar * obj.geom.S_canard * (K_WB+K_BW) * CN_c;
                HM_c(i)  = qbar * obj.geom.S_canard * obj.geom.c_canard * K_WB * (-0.05 * alpha_eff_c(i));

                CN_t = 0.1 * alpha_eff_t(i);
                F_N_t(i) = qbar * obj.geom.S_tail * (K_WB+K_BW) * CN_t;
                HM_t(i)  = qbar * obj.geom.S_tail * obj.geom.c_tail * K_WB * (-0.05 * alpha_eff_t(i));
            end

            % Body Alone & Synthesis
            alpha_cg = atan2(w, u);
            beta_cg = asin(v/V_total);
            F_X_b = qbar * obj.geom.S_ref * -0.2;
            F_Y_b = qbar * obj.geom.S_ref * (-0.05 * beta_cg);
            F_Z_b = qbar * obj.geom.S_ref * (-0.05 * alpha_cg);

            F_aero = [F_X_b;
                F_Y_b + (F_N_c(1)+F_N_c(3)) + (F_N_t(1)+F_N_t(3));
                F_Z_b - (F_N_c(2)+F_N_c(4)) - (F_N_t(2)+F_N_t(4))];

            M_aero = [(F_N_c(4)-F_N_c(2))*R + (F_N_c(1)-F_N_c(3))*R + (F_N_t(4)-F_N_t(2))*R + (F_N_t(1)-F_N_t(3))*R;
                -(F_N_c(2)+F_N_c(4))*xc - (F_N_t(2)+F_N_t(4))*xt;
                (F_N_c(1)+F_N_c(3))*xc + (F_N_t(1)+F_N_t(3))*xt];

            obj.HingeMoments = [HM_c; HM_t; zeros(4,1)];
        end
    end
end