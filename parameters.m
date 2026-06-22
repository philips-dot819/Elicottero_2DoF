% Script di inizializzazione parametri per Elicottero 2-DOF
% Eseguire prima di avviare il modello Simulink
clear all; close all; clc;

%% Parametri Fisici Nominali
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

% Vettore parametri
helicopter_params = [J_alpha; J_y; J_z; I_b; m; l; c_alpha; c_beta; eps_p; eps_y; g];

% Condizioni iniziali
x0 = [0; 0; 0; 0]; % Stato iniziale del sistema
% x0 = [pi/6; 0.02; pi/8; -0.03]; % Stato iniziale del sistema

%% Parametri Fisici Incerti
% Incertezze
unc_m = 0.01;   % Incertezza massa (1%)
unc_l = 0.01;   % Incertezza lunghezza (1%)
unc_J = 0.05;   % Incertezza inerzie (5%)
unc_c = 0.07;   % Incertezza attriti (7%)
unc_eps = 0.07; % Incertezza aerodinamica (7%)

% Generazione parametri incerti
uncert = @(nom, unc) nom * ( (1 - unc) + (2 * unc) * rand() );

J_alpha_unc = uncert(J_alpha, unc_J); % Inerzia di pitch incerta [kg*m^2]
J_y_unc = uncert(J_y, unc_J);         % Inerzia rotore incerta [kg*m^2]
J_z_unc = uncert(J_z, unc_J);         % Inerzia rotore incerta [kg*m^2]
I_b_unc = uncert(I_b, unc_J);         % Inerzia corpo base incerta [kg*m^2]
m_unc = uncert(m, unc_m);             % Massa dell'elicottero incerta [kg]
l_unc = uncert(l, unc_l);             % Distanza dal perno al baricentro incerta [m]
c_alpha_unc = uncert(c_alpha, unc_c); % Coefficiente di attrito viscoso pitch incerto [N*m*s/rad]
c_beta_unc = uncert(c_beta, unc_c);   % Coefficiente di attrito viscoso yaw incerto [N*m*s/rad]
eps_p_unc = uncert(eps_p, unc_eps);   % Coefficiente di cross-thrust su pitch incerto
eps_y_unc = uncert(eps_y, unc_eps);   % Coefficiente di cross-thrust su yaw incerto
g_unc = g;                            % Accelerazione di gravità (invariata) [m/s^2]

% Vettore parametri incerti
% helicopter_params_unc = [J_alpha; J_y; J_z; I_b; m; l; c_alpha; c_beta; eps_p; eps_y; g];
helicopter_params_unc = [J_alpha_unc; J_y_unc; J_z_unc; I_b_unc; m_unc; l_unc; c_alpha_unc; c_beta_unc; eps_p_unc; eps_y_unc; g_unc];

%% Parametri Sensori
% Telecamera
f_cam = 50;          % Frequenza telecamera [Hz]
Ts_cam = 1/f_cam;    % Tempo di campionamento telecamera [s]
std_dev_cam = 0.005; % Precisione telecamera [m]

% Accelerometro
f_acc = 100;       % Frequenza accelerometro [Hz]
Ts_acc = 1/f_acc;  % Tempo di campionamento accelerometro [s]
std_dev_acc = 0.1; % Precisione accelerometro [m/s^2]

% Parametri Outlier
prob_outlier = 0.01;    % Probabilità di generazione outlier (1%)
amp_outlier_cam = 1.0;  % Ampiezza massima outlier telecamera [m]
amp_outlier_acc = 10.0; % Ampiezza massima outlier accelerometro [m/s^2]

%% Parametri Estended Kalman Filter EKF
dt_EKF = 0.001; % Tempo di campionamento del filtro [s]

% Matrice di Covarianza del Rumore di Processo
% Q_EKF = diag([1e-7, 1e-5, 1e-7, 1e-5]); % okay per modello perfetto e std_dev_cam = 0.5 e std_dev_acc = 1.0
% Q_EKF = diag([1e-6, 1e-3, 1e-6, 1e-3]); % molto buono (con std_dev_camm = 0.05, std_dev_accc = 0.5); troppo rumore forse se ho qualche disturbo (al momento ho solo noise in ingresso ed è top)?
Q_EKF = diag([1e-6, 1e-4, 1e-6, 1e-4]); % ottimo (con std_dev_camm = 0.05, std_dev_accc = 0.5) ma molto buono anche con std_dev_camm = 0.005, std_dev_accc = 0.1
% Q_EKF = diag([1e-5, 1e-3, 1e-5, 1e-3]); % un po' troppo rumore (con std_dev_camm = 0.05, std_dev_accc = 0.5)

% Matrice di Covarianza del Rumore di Misura
% std_dev_cam = 0.05; % Precisione telecamera [m]
% std_dev_acc = 0.5;  % Precisione accelerometro [m/s^2]
R_EKF = diag([std_dev_cam^2, std_dev_cam^2, std_dev_acc^2]);

% Soglia attivazione controllo Mahalanobis per outlier
threshold_maha = 4;

% Condizioni Iniziali
x0_EKF = [0; 0; 0; 0]; % Stato iniziale ipotizzato nel filtro
% x0_EKF = [pi/2; 0.2; pi/4; -0.15]; % Stato iniziale ipotizzato nel filtro
P0_EKF = diag([1, 1, 1, 1]); % Matrice di covarianza iniziale (incertezza alta)
% P0_EKF = diag([1e-2, 1e-2, 1e-2, 1e-2]); % Matrice di covarianza iniziale (incertezza bassa)
% P0_EKF = diag([10, 10, 10, 10]); % Matrice di covarianza iniziale (incertezza altissima)
y_meas0_EKF = [0; 0; 0]; % Memoria iniziale dei sensori

%% Parametri Particle Filter (PF)
fmax_PF = lcm(f_acc, f_cam);     % Frequenza massima del sistema [Hz]
dt_PF = 1 / fmax_PF;             % Passo temporale di integrazione del PF [s]
acc_period_PF = fmax_PF / f_acc; % Tick del solver per aggiornare l'accelerometro
cam_period_PF = fmax_PF / f_cam; % Tick del solver per aggiornare la telecamera

% Parametri Algoritmici
N_PF = 1000;                  % Numero totale di particelle
W0_PF = ones(1, N_PF) / N_PF; % Pesi iniziali uniformi e normalizzati
resampling_th_PF = 0.7;       % Soglia per attivare il resampling (N_eff < soglia * N)

% Matrice di Covarianza del Rumore di Processo
% Q_PF = diag([1e-6, 1e-4, 1e-6, 1e-4]);
% Q_PF = diag([1e-6, 1e-3, 1e-6, 1e-3]);
Q_PF = diag([1e-6, 1e-2, 1e-6, 1e-2]);

% Matrice di Covarianza del Rumore di Misura
R_PF = diag([std_dev_cam^2, std_dev_cam^2, std_dev_acc^2]);

% Condizioni Iniziali
var_alpha_PF = (2 * pi/180)^2; % Incertezza iniziale su pitch [rad^2]
var_beta_PF = (2 * pi/180)^2;  % Incertezza iniziale su yaw [rad^2]
var_dot_PF = 1;                % Incertezza iniziale sulle velocità [rad^2/s^2]

X0_PF = zeros(4, N_PF);
X0_PF(1, :) = (rand(1, N_PF) - 0.5) * pi;   % Distribuzione iniziale uniformemente per alpha tra -pi/2 e pi/2
X0_PF(2, :) = (rand(1, N_PF) - 0.5) * 2;    % Distribuzione iniziale uniformemente per alpha_dot tra -1 rad/s e +1 rad/s
X0_PF(3, :) = (rand(1, N_PF) - 0.5) * 2*pi; % Distribuzione iniziale uniformemente per beta tra -pi e pi
X0_PF(4, :) = (rand(1, N_PF) - 0.5) * 2;    % Distribuzione iniziale uniformemente per beta_dot tra -1 rad/s e +1 rad/s

% Parametri per il Roughening (Jittering) post-resampling
sigma_jitter_alpha_PF = deg2rad(0.5); % [rad]
sigma_jitter_beta_PF = deg2rad(0.5); % [rad]

sigma_jitter_d_alpha_PF = 0.05; % [rad/s]
sigma_jitter_d_beta_PF = 0.05; % [rad/s]

% Matrice di Covarianza Q_jitter
Q_jitter_PF = diag([sigma_jitter_alpha_PF^2, sigma_jitter_d_alpha_PF^2, sigma_jitter_beta_PF^2,  sigma_jitter_d_beta_PF^2]);