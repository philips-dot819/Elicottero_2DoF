% =========================================================================
% PARAMETRI NOMINALI DELL'ELICOTTERO 2-DoF
% Corso di Identificazione, Stima e Controllo Robusto (2026)
% Modulo di Identificazione e Stima
% =========================================================================

clear; clc;

%% 1. Parametri Strutturali e Geometrici (Tabella 1)
J_alpha = 0.012;    % Inerzia di pitch [kg*m^2] (Tabella 1)
J_y     = 0.00023;  % Componente inerzia di yaw Y [kg*m^2] (Tabella 1)
J_z     = 0.00364;  % Componente inerzia di yaw Z [kg*m^2] (Tabella 1)
I_b     = 0.00023;  % Inerzia della base [kg*m^2] (Tabella 1)
m       = 0.2;      % Massa dell'elicottero [kg] (Tabella 1)
l       = 0.2;      % Lunghezza del braccio [m] (Tabella 1)
g       = 9.81;     % Accelerazione di gravità [m/s^2] (Tabella 1)

%% 2. Coefficienti di Attrito (Tabella 1)
c_alpha = 0.01;     % Coefficiente d'attrito viscoso pitch [N*m*s/rad] (Tabella 1)
c_beta  = 0.01;     % Coefficiente d'attrito viscoso yaw [N*m*s/rad] (Tabella 1)

%% 3. Coefficienti Aerodinamici e Cross-Thrust (Tabella 1)
eps_p   = 0.1;      % Effetto cross-thrust su pitch (Tabella 1)
eps_y   = 0.1;      % Effetto cross-thrust su yaw (Tabella 1)

%% 4. Frequenze di Campionamento dei Sensori (Multi-rate Asincrono)
% Configurazione dei sample time per i blocchi Simulink dei sensori
Ts_Girosc    = 1 / 104;  % Passo IMU (~9.6 ms, basato su specifiche LSM6DSOX)
Ts_Altitude  = 1 / 30;   % Passo sensore di distanza (33.3 ms, VL53L1X in Short Mode)
Ts_Acceler   = 1 / 80;   


%% 5. Parametri sensore ultrasuoni per alpha
% Sensore ultrasuoni montato sotto il muso dell'elicottero.
% Il sensore misura la distanza verticale dal tavolo.


h0 = 0.30;                  % [m] altezza del perno rispetto al tavolo

sigma_alpha_sensor = 0.01;   % 1 cm; % [m] deviazione standard rumore sensore

Ts_ToF_alpha = 1 / 30;      % [s] sample time sensore ultrasuoni, 30 Hz

%% Parametri Vicon telecamera

% sensore posizionato sopra al perno che ricava la misura x e y della coda
% dell'elicottero

sigma_cam_sensor = 0.005;
Ts_Tof_cam       = 1/45;

%% Parametri per inizializzare il filtro a particelle

N = 10000;
W0 = ones(1, N)/N;
dt = 1/90;

% probabilità a priori per lo stato iniziale
% definita come gaussiana per ogni variabile di stato, media nulla.

var_alpha =  (2 * pi/180)^2;
var_beta  =  (2 * pi/180)^2;
var_dot   =  1;

X0 = zeros(4, N);
X0(1, :) = sqrt(var_alpha) * randn(1, N);  % Alpha iniziali
X0(2, :) = sqrt(var_beta) * randn(1, N);   % Beta iniziali
X0(3, :) = sqrt(var_dot) * randn(1, N);    % Alpha_dot
X0(4, :) = sqrt(var_dot) * randn(1, N);    % Beta_dot

%% Matrice di covarianza del rumore di misura.

R_sens = [sigma_alpha_sensor^2       0                    0;...
                 0          sigma_cam_sensor^2          0;...
                 0                0                 sigma_cam_sensor^2];

%% Matrice di covarianza del rumore di processo PARTICLE

% Definisci le deviazioni standard (sigma) attese per ogni step dt
sigma_alpha = 0.5 * (pi/180); % Incertezza di 0.5 gradi a step
sigma_beta  = 0.5 * (pi/180); % Incertezza di 0.5 gradi a step
sigma_d_alpha = 0.1;        % Incertezza di 0.1 rad/s a step sulle velocità
sigma_d_beta  = 0.1;

% Costruisci la matrice diagonale Q
Q_process = diag([sigma_alpha^2, sigma_beta^2, sigma_d_alpha^2, sigma_d_beta^2]);

%% Parametri resampling PARTICLE

Resampling_th = 0.5;

%%#### 20/06 ####

%% Accelerometro pitch
sigma_acc_sensor = 0.05;   % [m/s^2],rumore acc, valore iniziale da tarare
Ts_acceler = 1/1344;

%questi valori poi i tarano l'ho fatto per creare una struttura logica
%delle cose da fare

%% Covarianza misure EKF
R_EKF = diag([
    sigma_cam_sensor^2
    sigma_cam_sensor^2
    sigma_acc_sensor^2
]);

%% Parametri EKF
Ts_EKF = 1/104;  % oppure scegliamo il rate principale del filtro

x0_EKF = [0; 0; 0; 0];   % [alpha; d_alpha; beta; d_beta]

P0_EKF = diag([
    1^2
    1^2
    1^2
    1^2
    ]);

Q_EKF = diag([
    1e-5
    1e-3
    2e-6
    2e-5
]);

disp('=== Parametri nominali caricati nel Workspace con successo ===');