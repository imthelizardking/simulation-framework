function [alpha_e, aero_accs,  va_dot, spin_acc, alpha_e_mag, I_x_t] = calc_accs(pos, ro, Va, va_b, m, CDO_reduce, FITTING_FACTORS, coeffs, v_b, alpha_e_mag_old, I_x, spin_old, INIT_AZ, INIT_LAT, MUN_CG_INIT, MUN_INIT_INERTIA, MUN_INIT_MASS, SIM_CORIOLIS, BASEBLEED_ON, BASEBLEED_CG_x_BURNOUT, BASEBLEED_I_x_b_burnout, BASEBLEED_MFUEL, MUN_D, MUN_S)
%% dinamik değişkenleri hesaplar
% v0: 25.11.06 yco
% versiyon notları:
% ilk versiyon
% girdiler:
% Va (1x1): aerodinamik hız
% ro (1x1): bkz. calc_Metro
% FITTING_FACTORS (4x1): iform, fL, QD, QM
% m (1x1): kütle
% coeffs (15x1): aerodinamik katsayılar (birimsiz)
% MUN_S (1x1): mühimmat kesit alanı (m^2)
% D (1x1): mühimmat referans çapı (m)
% TIME_STEP (1x1): simülasyon zaman adımı (s)
% X_CG_B (1x1): basebleed
% X_CG_0 (1x1): basebleed
% M_INIT (1x1): basebleed
% BASEBLEED_MFUEL (1x1): basebleed
% I_x_0 (1x1): başlangıçtaki eksenel atalet (kg/m^3)
% I_x_B (1x1): basebleed
% SIM_CORIOLIS (1x1): SIM_CORIOLIS on/off (boolean)
% INIT_LAT (1x1): atışın yapıldığı enlem (°)
% va_b (3x1): mühimmat aerodinamik hızı (yere göre) (m/s)
% CD0_reduce (1x1): basebleed
% v_b (3x1): mühimmat hızı (yere göre) (m/s)
% spin_old (1x1): dönü (rad/sec)
% alpha_e_old (3x1): yaw of repose (rad)
% I_x_ (1x1): değişken eksenel atalet (kg/m^3)
% çıktılar:
% va_dot (3x1): body ivmeleri (yere göre) (m/s^2)
% spin_acc (1x1): dönü ivmesi (rad/sec^2)
% alpha_e (3x1): yaw of repose (rad)
% I_x_t (1x1): değişken eksenel atalet (kg/m^3)
%% base-bleed calculations (mass and CG change)
X_CG_t = MUN_CG_INIT;
I_x_t = MUN_INIT_INERTIA;
if BASEBLEED_ON
    M_B = MUN_INIT_MASS-BASEBLEED_MFUEL;
    X_CG_f0 = BASEBLEED_CG_x_BURNOUT + (MUN_CG_INIT-BASEBLEED_CG_x_BURNOUT)*MUN_INIT_MASS/(MUN_INIT_MASS-M_B);
    X_CG_t = BASEBLEED_CG_x_BURNOUT + (X_CG_f0-BASEBLEED_CG_x_BURNOUT)*(m-M_B)/m;
    I_x_t = MUN_INIT_INERTIA + (MUN_INIT_INERTIA-BASEBLEED_I_x_b_burnout)*(m-MUN_INIT_MASS)/(MUN_INIT_MASS-(MUN_INIT_MASS-BASEBLEED_MFUEL));
end
%%
% get form factors
i_form = FITTING_FACTORS(1); f_l = FITTING_FACTORS(2);
QD = FITTING_FACTORS(3); QM = FITTING_FACTORS(4);


% get aero coeffs.:
CD0 = coeffs(1);
CDA2 = coeffs(2);
CDA4 = coeffs(3);
CD_total = max(CD0 + CDA2*(QD*alpha_e_mag_old)^2 + CDA4*(QD*alpha_e_mag_old)^4 -abs(CDO_reduce),0);
CLA = coeffs(4); CLA3 = coeffs(5); CLA5 = coeffs(6);
CLA_total = CLA + CLA3*alpha_e_mag_old^2 + CLA5*alpha_e_mag_old^4;
CMAGF = coeffs(9);
CSPIN = coeffs(11);
CMA = coeffs(7);
CMA3 = coeffs(8);
CMA = CMA + (X_CG_t-MUN_CG_INIT)*(CD0-CLA)/MUN_D; % calculate active CMA (due to base-bleed, changing CG)
CMA3 = CMA3 + (X_CG_t-MUN_CG_INIT)*(CLA3+CDA2-0.5*CLA)/MUN_D; % calculate active CMA3 (due to base-bleed, changing CG)
CMA_total = CMA + CMA3*alpha_e_mag_old^2;
%% calculate gravity
% % method1:
% R = 6.356766*10^6;
% g0 = 9.8065*(1-0.0026*cosd(2*INIT_LAT));
% X1 = init_x; X2 = init_z; X3 = init_y;
% acc_gravity = -g0*[X1/R; 1-2*X2/R; X3/R];
% % method 2:
% g0 = 9.80665; % mean
% g = g0*(1-0.0026*cosd(2*INIT_LAT));
% acc_gravity = g*[0; -1; 0];
% method 3:
g0 = 9.80665; % mean
g = g0*(1-0.0026*cosd(2*INIT_LAT));
R = 6.356766E+06;
r = pos - [0; -R; 0];
acc_gravity = -g*(R^2/(sqrt(r'*r))^3)*r;
%%
% calculate SIM_CORIOLIS
Ohm = 7.292115*10^(-5); % Angular speed of the Earth (rad/s)
v_Omega = [Ohm*cosd(INIT_LAT)*cosd(INIT_AZ*pi/3200) ; Ohm*sind(INIT_LAT) ; -Ohm*cosd(INIT_LAT)*sin(INIT_AZ*pi/3200)];
acc_coriolis = [0; 0; 0];
if SIM_CORIOLIS==true
    acc_coriolis = -2*cross(v_Omega,v_b);
end
%%
% method 1: direct approach, no fixed-point iter
% 4355 formula:
% calculate yaw of repose and spin acceleration and spin
% alpha_e = -(2 * I_x_t * spin_old * cross(va_b, va_dot_in)) / (ro*MUN_S*D*CMA_total*Va^4);
% bradley's formula:
alpha_e = (2*I_x_t*spin_old*cross(acc_gravity,va_b))/(ro*MUN_S*MUN_D*CMA_total*Va^4);
alpha_e_mag = sqrt(alpha_e'*alpha_e);
spin_acc = (pi*ro*MUN_D^4*spin_old*Va*CSPIN) / (8*I_x_t);
% calculate accelerations
acc_drag = -(MUN_S*ro*i_form/2)*CD_total*Va*va_b/m;
acc_lift = (MUN_S*ro*f_l/2)*CLA_total*Va^2*alpha_e/m;
acc_magnus = -(MUN_S*ro*MUN_D*QM*spin_old*CMAGF)*cross(alpha_e, va_b)/m/2;
va_dot = acc_drag + acc_lift + acc_magnus + acc_gravity + acc_coriolis;

aero_accs = [acc_drag; acc_lift; acc_magnus];
%% method 2: fixed point iter
% % 4355 formula:
% % calculate yaw of repose and spin acceleration and spin
% % alpha_e = -(2 * I_x_t * spin_old * cross(va_b, va_dot_in)) / (ro*MUN_S*D*CMA_total*Va^4);
% % bradley's formula:
% alpha_e = (2 * I_x_t * spin_old * cross(acc_gravity, va_b)) / (ro * MUN_S * D * CMA_total * Va^4);
% 
% spin_acc = (pi*ro*D^4*spin_old*Va*CSPIN) / (8*I_x_t);
% % spin = spin_old + spin_acc*TIME_STEP;
% spin = spin_old;
% 
% % calculate accelerations
% acc_drag = -(MUN_S*ro*i_form/2)*CD_total*Va*va_b/m;
% acc_lift = (MUN_S*ro*f_l/2)*CLA_total*Va^2*alpha_e/m;
% acc_magnus = -(MUN_S*ro*D*QM*spin*CMAGF)*cross(alpha_e, va_b)/m/2;
% va_dot = acc_drag + acc_lift + acc_magnus + acc_gravity + acc_coriolis;
% % va_b = va_b + va_dot*TIME_STEP;
% Va = sqrt(va_b'*va_b);
% 
% % run fixed-point iteration
% tol_alpha_e = 10^(-8);
% diff_alpha_e = alpha_e - alpha_e_old;
% diff_alpha_e = sqrt(diff_alpha_e'*diff_alpha_e);
% alpha_e_apprx = alpha_e;
% nCounter = 1;
% while diff_alpha_e>tol_alpha_e && nCounter<100
% 
%     alpha_e_apprx_old = alpha_e_apprx;
%     alpha_e_apprx_mag = sqrt(alpha_e_apprx_old'*alpha_e_apprx_old);
% 
%     CD_total = max(CD0 + CDA2*(QD*alpha_e_apprx_mag)^2 + CDA4*(QD*alpha_e_apprx_mag)^4-abs(CDO_reduce),0);
%     CLA_total = CLA + CLA3*alpha_e_old_mag^2 + CLA5*alpha_e_old_mag^4;
%     CMA_total = CMA + CMA3*alpha_e_old_mag^2;
% 
%     % calculate accelerations
%     acc_drag = -(MUN_S*ro*i_form/2)*CD_total*Va*va_b/m;
%     acc_lift = (MUN_S*ro*f_l/2)*CLA_total*Va^2*alpha_e/m;
%     acc_magnus = -(MUN_S*ro*D*QM*spin*CMAGF)*cross(alpha_e, va_b)/m/2;
%     va_dot = acc_drag + acc_lift + acc_magnus + acc_gravity + acc_coriolis;
% 
%     % k1 = va_dot;
%     % k2 = ((va_b+TIME_STEP/2*k1))*va_dot;
%     % k3 = ((va_b+TIME_STEP/2*k2))*va_dot;
%     % k4 = ((va_b+TIME_STEP/2*k3))*va_dot;
%     % va_b_test = va_b + TIME_STEP/6*(k1+2*k2+2*k3+k4);
% 
%     % va_b = va_b + va_dot*TIME_STEP;
%     % Va = sqrt(va_b'*va_b);
% 
%     % calculate new spin
%     spin_acc = (pi*ro*D^4*spin*Va*CSPIN) / (8*I_x);
%     % spin = spin + spin_acc*TIME_STEP;
% 
%     % calculate alpha_e and diff
%     % 4355 formula:
%     % alpha_e_apprx = -(2 * I_x_t * spin_old * cross(va_b, va_dot)) / (ro * MUN_S * D * CMA_total * Va^4);
%     % bradley's formula:
%     alpha_e_apprx = (2 * I_x_t * spin_old * cross(acc_gravity, va_b)) / (ro * MUN_S * D * CMA_total * Va^4);
%     diff_alpha_e = sqrt((alpha_e_apprx - alpha_e_apprx_old)'*(alpha_e_apprx - alpha_e_apprx_old));
%     nCounter = nCounter + 1;
% end
% alpha_e = alpha_e_apprx;
end