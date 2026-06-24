%% ============================================================
%  Script di inizializzazione parametri per Elicottero 2-DOF
%  Eseguire prima di avviare il modello Simulink
%% ============================================================

clear all; close all; clc;

%% ============================================================
%  0. FLAG DI SIMULAZIONE
%% ============================================================

% Per ripetibilità simulazioni
flag_fixed_seed = true;
rng_seed = 8;

% Parametri fisici plant
% "nominali" oppure "incerti"
plant_param_case = "incerti";

% Condizione iniziale reale del modello
% "zero" oppure "perturbata"
x0_case = "perturbata";

% Rumore sensori
% "basso", "nominale", "alto"
sensor_noise_case = "basso";

% Outlier
flag_use_outliers = true;

% EKF
% "bassa", "nominale", "alta"
Q_EKF_case = "nominale";

% "corretta", "errata_moderata", "errata_forte"
x0_EKF_case = "corretta";

% "bassa", "alta", "altissima"
P0_EKF_case = "alta";

% PF
% "nominale", "velocita_alta"
Q_PF_case = "velocita_alta";

% "ampia", "stretta_attorno_x0"
X0_PF_case = "ampia";

% Numero particelle
% puoi cambiare solo questo numero quando vuoi fare prove rapide/pesanti
N_PF_case = 1000;

% Soglie usate nello script metriche
tol = [0.05, 0.30, 0.05, 0.30];

if flag_fixed_seed
    rng(rng_seed);
end

%% ============================================================
%  1. Parametri Fisici Nominali
%% ============================================================

J_alpha = 0.012; % Inerzia di pitch [kg*m^2]
J_y = 0.00023;   % Inerzia rotore [kg*m^2]
J_z = 0.00364;   % Inerzia rotore [kg*m^2]
I_b = 0.00023;   % Inerzia corpo base [kg*m^2]
m = 0.2;         % Massa dell'elicottero [kg]
l = 0.2;         % Distanza dal perno al baricentro [m]
c_alpha = 0.01;  % Coefficiente di attrito viscoso pitch [N*m*s/rad]
c_beta = 0.01;   % Coefficiente di attrito viscoso yaw [N*m*s/rad]
eps_p = 0.1;     % Coefficiente di cross-thrust su pitch
eps_y = 0.1;     % Coefficiente di cross-thrust su yaw
g = 9.81;        % Accelerazione di gravità [m/s^2]

helicopter_params = [
    J_alpha
    J_y
    J_z
    I_b
    m
    l
    c_alpha
    c_beta
    eps_p
    eps_y
    g
];

%% ============================================================
%  2. Condizioni iniziali reali del modello
%% ============================================================

switch x0_case

    case "zero"
        x0 = [0; 0; 0; 0];

    case "perturbata"
        x0 = [pi/6; 0.02; pi/8; -0.03];

    otherwise
        error('x0_case non riconosciuto: %s', x0_case);

end

%% ============================================================
%  3. Parametri Fisici Incerti
%% ============================================================

unc_m = 0.01;   % Incertezza massa (1%)
unc_l = 0.01;   % Incertezza lunghezza (1%)
unc_J = 0.05;   % Incertezza inerzie (5%)
unc_c = 0.07;   % Incertezza attriti (7%)
unc_eps = 0.07; % Incertezza aerodinamica (7%)

uncert = @(nom, unc) nom * ((1 - unc) + (2 * unc) * rand());

J_alpha_unc = uncert(J_alpha, unc_J);
J_y_unc = uncert(J_y, unc_J);
J_z_unc = uncert(J_z, unc_J);
I_b_unc = uncert(I_b, unc_J);
m_unc = uncert(m, unc_m);
l_unc = uncert(l, unc_l);
c_alpha_unc = uncert(c_alpha, unc_c);
c_beta_unc = uncert(c_beta, unc_c);
eps_p_unc = uncert(eps_p, unc_eps);
eps_y_unc = uncert(eps_y, unc_eps);
g_unc = g;

helicopter_params_unc_generated = [
    J_alpha_unc
    J_y_unc
    J_z_unc
    I_b_unc
    m_unc
    l_unc
    c_alpha_unc
    c_beta_unc
    eps_p_unc
    eps_y_unc
    g_unc
];

% Mantengo il nome helicopter_params_unc per compatibilità con Simulink.
% Se plant_param_case = "nominali", anche helicopter_params_unc viene posto uguale ai nominali.
switch plant_param_case

    case "nominali"
        helicopter_params_unc = helicopter_params;

    case "incerti"
        helicopter_params_unc = helicopter_params_unc_generated;

    otherwise
        error('plant_param_case non riconosciuto: %s', plant_param_case);

end

%% ============================================================
%  4. Parametri Sensori
%% ============================================================

f_cam = 50;
Ts_cam = 1/f_cam;

f_acc = 100;
Ts_acc = 1/f_acc;

switch sensor_noise_case

    case "basso"
        std_dev_cam = 0.005; % [m]
        std_dev_acc = 0.1;   % [m/s^2]

    case "nominale"
        std_dev_cam = 0.05;  % [m]
        std_dev_acc = 0.5;   % [m/s^2]

    case "alto"
        std_dev_cam = 0.10;  % [m]
        std_dev_acc = 1.0;   % [m/s^2]

    otherwise
        error('sensor_noise_case non riconosciuto: %s', sensor_noise_case);

end

%% ============================================================
%  5. Parametri Outlier
%% ============================================================

if flag_use_outliers

    prob_outlier = 0.01;
    amp_outlier_cam = 1.0;
    amp_outlier_acc = 10.0;

else

    prob_outlier = 0;
    amp_outlier_cam = 0;
    amp_outlier_acc = 0;

end

%% ============================================================
%  6. Parametri Extended Kalman Filter EKF
%% ============================================================

dt_EKF = 0.001;

switch Q_EKF_case

    case "bassa"
        Q_EKF = diag([1e-7, 1e-5, 1e-7, 1e-5]);

    case "nominale"
        Q_EKF = diag([1e-6, 1e-4, 1e-6, 1e-4]);

    case "alta"
        Q_EKF = diag([1e-5, 1e-3, 1e-5, 1e-3]);

    otherwise
        error('Q_EKF_case non riconosciuto: %s', Q_EKF_case);

end

R_EKF = diag([
    std_dev_cam^2
    std_dev_cam^2
    std_dev_acc^2
]);

threshold_maha = 4;

switch x0_EKF_case

    case "corretta"
        x0_EKF = [0; 0; 0; 0];

    case "errata_moderata"
        x0_EKF = [pi/12; 0.05; pi/12; -0.05];

    case "errata_forte"
        x0_EKF = [pi/2; 0.2; pi/4; -0.15];

    otherwise
        error('x0_EKF_case non riconosciuto: %s', x0_EKF_case);

end

switch P0_EKF_case

    case "bassa"
        P0_EKF = diag([1e-2, 1e-2, 1e-2, 1e-2]);

    case "alta"
        P0_EKF = diag([1, 1, 1, 1]);

    case "altissima"
        P0_EKF = diag([10, 10, 10, 10]);

    otherwise
        error('P0_EKF_case non riconosciuto: %s', P0_EKF_case);

end

y_meas0_EKF = [0; 0; 0];

%% ============================================================
%  7. Parametri Particle Filter PF
%% ============================================================

fmax_PF = lcm(f_acc, f_cam);
dt_PF = 1 / fmax_PF;
acc_period_PF = fmax_PF / f_acc;
cam_period_PF = fmax_PF / f_cam;

N_PF = N_PF_case;
W0_PF = ones(1, N_PF) / N_PF;
resampling_th_PF = 0.7;

switch Q_PF_case

    case "nominale"
        Q_PF = diag([1e-6, 1e-4, 1e-6, 1e-4]);

    case "velocita_alta"
        Q_PF = diag([1e-6, 1e-2, 1e-6, 1e-2]);

    otherwise
        error('Q_PF_case non riconosciuto: %s', Q_PF_case);

end

R_PF = diag([
    std_dev_cam^2
    std_dev_cam^2
    std_dev_acc^2
]);

var_alpha_PF = (2 * pi/180)^2;
var_beta_PF = (2 * pi/180)^2;
var_dot_PF = 1;

X0_PF = zeros(4, N_PF);

switch X0_PF_case

    case "ampia"

        X0_PF(1, :) = (rand(1, N_PF) - 0.5) * pi;
        X0_PF(2, :) = (rand(1, N_PF) - 0.5) * 2;
        X0_PF(3, :) = (rand(1, N_PF) - 0.5) * 2*pi;
        X0_PF(4, :) = (rand(1, N_PF) - 0.5) * 2;

    case "stretta_attorno_x0"

        X0_PF(1, :) = x0(1) + sqrt(var_alpha_PF) * randn(1, N_PF);
        X0_PF(2, :) = x0(2) + sqrt(var_dot_PF)   * randn(1, N_PF);
        X0_PF(3, :) = x0(3) + sqrt(var_beta_PF)  * randn(1, N_PF);
        X0_PF(4, :) = x0(4) + sqrt(var_dot_PF)   * randn(1, N_PF);

    otherwise
        error('X0_PF_case non riconosciuto: %s', X0_PF_case);

end

X0_PF(1,:) = wrapToPi(X0_PF(1,:));
X0_PF(3,:) = wrapToPi(X0_PF(3,:));

sigma_jitter_alpha_PF = deg2rad(0.5);
sigma_jitter_beta_PF = deg2rad(0.5);

sigma_jitter_d_alpha_PF = 0.05;
sigma_jitter_d_beta_PF = 0.05;

Q_jitter_PF = diag([
    sigma_jitter_alpha_PF^2
    sigma_jitter_d_alpha_PF^2
    sigma_jitter_beta_PF^2
    sigma_jitter_d_beta_PF^2
]);

%% ============================================================
%  8. Riepilogo configurazione
%% ============================================================

config_sim = struct();

config_sim.flag_fixed_seed = flag_fixed_seed;
config_sim.rng_seed = rng_seed;
config_sim.plant_param_case = plant_param_case;
config_sim.x0_case = x0_case;
config_sim.sensor_noise_case = sensor_noise_case;
config_sim.flag_use_outliers = flag_use_outliers;
config_sim.Q_EKF_case = Q_EKF_case;
config_sim.x0_EKF_case = x0_EKF_case;
config_sim.P0_EKF_case = P0_EKF_case;
config_sim.Q_PF_case = Q_PF_case;
config_sim.X0_PF_case = X0_PF_case;
config_sim.N_PF_case = N_PF_case;
config_sim.tol = tol;

disp(' ');
disp('========== CONFIGURAZIONE SIMULAZIONE ==========');
disp(config_sim);

disp(' ');
disp('========== PARAMETRI PRINCIPALI ==========');
fprintf('Plant parametri      : %s\n', plant_param_case);
fprintf('x0 reale             : %s\n', mat2str(x0', 4));
fprintf('Rumore sensori       : %s\n', sensor_noise_case);
fprintf('Outlier attivi       : %d\n', flag_use_outliers);
fprintf('Q_EKF case           : %s\n', Q_EKF_case);
fprintf('x0_EKF case          : %s\n', x0_EKF_case);
fprintf('P0_EKF case          : %s\n', P0_EKF_case);
fprintf('Q_PF case            : %s\n', Q_PF_case);
fprintf('X0_PF case           : %s\n', X0_PF_case);
fprintf('N_PF                 : %d\n', N_PF);
fprintf('tol                  : %s\n', mat2str(tol, 4));
fprintf('std_dev_cam          : %.3e m\n', std_dev_cam);
fprintf('std_dev_acc          : %.3e m/s^2\n', std_dev_acc);

