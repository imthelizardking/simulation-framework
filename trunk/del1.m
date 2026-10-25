% =========================================================================
% SENTETİK VERİ İLE AERO KATSAYI OPTİMİZASYON TESTİ
% (Komut penceresine doğrudan yapıştırılmaya uygun REPL versiyonu)
% =========================================================================

% 1. Paketleri ve Globalleri Tanımla
pkg load optim;
global mach_nodes radar_data;

% =========================================================================
% ÖNCE YARDIMCI FONKSİYONLAR TANIMLANIR (Octave'ın öğrenmesi için)
% =========================================================================

% --- Basitleştirilmiş 6-DOF / MPMM Modeli ---
function sim = run_mpmm_dummy(i_drag, mach_nodes, c_spin, c_magf)
    t = linspace(0, 30, 100)';
    dt = t(2) - t(1);
    mach = linspace(2.0, 0.8, 100)';

    inst_c_spin = interp1(mach_nodes, c_spin, mach, 'spline', 'extrap');
    inst_c_magf = interp1(mach_nodes, c_magf, mach, 'spline', 'extrap');

    X = zeros(100,1); Y = zeros(100,1); Z = zeros(100,1); spin = zeros(100,1);

    X(1) = 0; Y(1) = 0; Z(1) = 0; spin(1) = 280;
    Vx = 600 / i_drag;
    Vy = 300;
    Vz = 0;

    for k = 2:100
        Vx = Vx - (0.015 * i_drag * Vx) * dt;
        Vy = Vy - 9.81 * dt;
        Vz = Vz + (inst_c_magf(k) * 40) * dt;

        X(k) = X(k-1) + Vx * dt;
        Y(k) = Y(k-1) + Vy * dt;
        Z(k) = Z(k-1) + Vz * dt;

        spin(k) = spin(k-1) - (inst_c_spin(k) * spin(k-1)) * dt;
    end
    sim.time = t; sim.X = X; sim.Y = Y; sim.Z = Z; sim.spin = spin;
end

% --- Sentetik Veri Üreticisi ---
function radar = generate_synthetic_radar()
    global mach_nodes;
    true_drag = 1.35;
    true_spin = [0.015, 0.018, 0.022, 0.025, 0.028];
    true_magf = [-0.04, -0.01, 0.06, 0.10, 0.15];

    perfect_flight = run_mpmm_dummy(true_drag, mach_nodes, true_spin, true_magf);

    idx = 1:3:100;
    radar.time = perfect_flight.time(idx);

    radar.X = perfect_flight.X(idx) + randn(length(idx), 1) * 20;
    radar.Y = perfect_flight.Y(idx) + randn(length(idx), 1) * 20;
    radar.Z = perfect_flight.Z(idx) + randn(length(idx), 1) * 3;
    radar.spin = perfect_flight.spin(idx) + randn(length(idx), 1) * 1.5;

    radar.true_drag = true_drag;
    radar.true_spin = true_spin;
    radar.true_mag  = true_magf;
end

% --- Maliyet Fonksiyonu (Residual Hesaplayıcı) ---
function residuals = cost_function(theta)
    global radar_data mach_nodes;

    i_drag = theta(1);
    c_spin = theta(2:6);
    c_magf = theta(7:11);

    sim_data = run_mpmm_dummy(i_drag, mach_nodes, c_spin, c_magf);

    sim_X    = interp1(sim_data.time, sim_data.X, radar_data.time, 'spline');
    sim_Y    = interp1(sim_data.time, sim_data.Y, radar_data.time, 'spline');
    sim_Z    = interp1(sim_data.time, sim_data.Z, radar_data.time, 'spline');
    sim_spin = interp1(sim_data.time, sim_data.spin, radar_data.time, 'spline');

    W_X = 0.05; W_Y = 0.05; W_Z = 1.0; W_S = 2.0;

    res_X = W_X * (sim_X - radar_data.X);
    res_Y = W_Y * (sim_Y - radar_data.Y);
    res_Z = W_Z * (sim_Z - radar_data.Z);
    res_S = W_S * (sim_spin - radar_data.spin);

    residuals = [res_X; res_Y; res_Z; res_S];
end

% --- Görselleştirme ---
function plot_results(theta_opt)
    global mach_nodes radar_data;

    c_spin_opt = theta_opt(2:6);
    c_magf_opt = theta_opt(7:11);
    mach_hires = linspace(0.8, 2.0, 100);

    spin_curve_opt = interp1(mach_nodes, c_spin_opt, mach_hires, 'spline');
    spin_curve_tru = interp1(mach_nodes, radar_data.true_spin, mach_hires, 'spline');
    magf_curve_opt = interp1(mach_nodes, c_magf_opt, mach_hires, 'spline');
    magf_curve_tru = interp1(mach_nodes, radar_data.true_mag, mach_hires, 'spline');

    figure;
    subplot(2,1,1);
    plot(mach_hires, spin_curve_tru, 'k--', 'LineWidth', 2); hold on;
    plot(mach_hires, spin_curve_opt, 'b-', 'LineWidth', 1.5);
    plot(mach_nodes, c_spin_opt, 'ro', 'MarkerFaceColor', 'r');
    legend('Gercek (Truth)', 'Tuning Sonucu', 'Kontrol Noktalari', 'Location', 'northwest');
    title('C_{SPIN} vs Mach Tuning Basarisi'); grid on;

    subplot(2,1,2);
    plot(mach_hires, magf_curve_tru, 'k--', 'LineWidth', 2); hold on;
    plot(mach_hires, magf_curve_opt, 'r-', 'LineWidth', 1.5);
    plot(mach_nodes, c_magf_opt, 'bo', 'MarkerFaceColor', 'b');
    legend('Gercek (Truth)', 'Tuning Sonucu', 'Kontrol Noktalari', 'Location', 'northwest');
    title('C_{MAGF} (Magnus) vs Mach Tuning Basarisi'); grid on;
end

% =========================================================================
% 2. ANA ÇALIŞTIRMA KODU (Artık tüm fonksiyonlar bellekte)
% =========================================================================

mach_nodes = [0.8, 0.95, 1.1, 1.5, 2.0];

fprintf('\nSentetik Doppler Radar verisi üretiliyor (gürültülü)...\n');
radar_data = generate_synthetic_radar();

i_drag_guess = 1.0;
c_spin_guess = [0.010, 0.010, 0.010, 0.010, 0.010];
c_magf_guess = [0.00, 0.00, 0.00, 0.00, 0.00];
theta_0 = [i_drag_guess, c_spin_guess, c_magf_guess];

lb = [0.5,  0.0, 0.0, 0.0, 0.0, 0.0,  -0.5, -0.5, -0.5, -0.5, -0.5];
ub = [2.0,  0.1, 0.1, 0.1, 0.1, 0.1,   0.5,  0.5,  0.5,  0.5,  0.5];

options = optimset('Algorithm', 'lm', 'Display', 'iter', 'TolFun', 1e-6, 'MaxIter', 150);

fprintf('\nOptimizasyon Başlıyor...\n');
[theta_opt, ~, ~, ~] = lsqnonlin(@cost_function, theta_0, lb, ub, options);

fprintf('\n=== SONUÇ KARŞILAŞTIRMASI ===\n');
fprintf('Drag (i):   Gercek = %.4f | Bulunan = %.4f\n', radar_data.true_drag, theta_opt(1));

disp('C_SPIN Gercek:');  disp(radar_data.true_spin);
disp('C_SPIN Bulunan:'); disp(theta_opt(2:6));

disp('C_MAGF Gercek:');  disp(radar_data.true_mag);
disp('C_MAGF Bulunan:'); disp(theta_opt(7:11));

plot_results(theta_opt);
