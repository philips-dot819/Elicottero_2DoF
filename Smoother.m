%% --- PASSO ALL'INDIETRO: RAUCH-TUNG-STRIEBEL (RTS) SMOOTHER ---
disp('--- INIZIO ESECUZIONE SCRIPT ---'); % Spia per verificare che parta

% 1. Estrazione e Correzione Forzata delle Dimensioni
x_corr = out.x_hat_out';                    % Trasposta: da 10001x4 a 4x10001
x_pred = reshape(out.x_prior_out, 4, []);   % Reshape forzato: da 4x1x10001 a 4x10001
P_corr = out.P_out;                         % 4x4x10001
P_pred = out.P_prior_out;                   % 4x4x10001
F_mat  = out.F_jacob_out;                   % 4x4x10001

% Estraiamo anche il vettore del tempo da 'out'!
t_out  = out.tout;                          

% Verifichiamo subito le dimensioni nel Command Window per sicurezza
disp(['Dimensione x_corr: ', num2str(size(x_corr,1)), 'x', num2str(size(x_corr,2))]);
disp(['Dimensione x_pred: ', num2str(size(x_pred,1)), 'x', num2str(size(x_pred,2))]);

% 2. Setup iniziale
N_steps = size(x_corr, 2); 
x_smooth = zeros(4, N_steps);
P_smooth = zeros(4, 4, N_steps);

% Inizializzazione 
x_smooth(:, N_steps)   = x_corr(:, N_steps);
P_smooth(:, :, N_steps) = P_corr(:, :, N_steps);

% 3. Ciclo all'indietro
for k = (N_steps - 1) : -1 : 1
    
    P_k_corr  = P_corr(:, :, k);         
    F_k       = F_mat(:, :, k);          
    P_k1_pred = P_pred(:, :, k+1);       
    
    % Calcolo del Guadagno di Smoothing
    C_k = P_k_corr * F_k' / P_k1_pred; 
    
    % Calcolo della "differenza" angolare
    dx = x_smooth(:, k+1) - x_pred(:, k+1);
    
    % WRAPPING DELLA DIFFERENZA (Il passaggio critico mancante!)
    dx(1) = wrapToPi(dx(1));
    dx(3) = wrapToPi(dx(3));
    
    % Aggiornamento dello Stato Lisciato usando la differenza corretta
    x_smooth(:, k) = x_corr(:, k) + C_k * dx;
    
    % Wrapping rigoroso dello stato finale
    x_smooth(1, k) = wrapToPi(x_smooth(1, k));
    x_smooth(3, k) = wrapToPi(x_smooth(3, k));
    
    % Aggiornamento della Covarianza Lisciata
    P_smooth(:, :, k) = P_k_corr + C_k * (P_smooth(:, :, k+1) - P_k1_pred) * C_k';
    
    % Mantenimento della simmetria
    P_smooth(:, :, k) = (P_smooth(:, :, k) + P_smooth(:, :, k)') / 2;
end

disp('RTS Smoothing calcolato. Generazione grafici in corso...');

%% --- VISUALIZZAZIONE RISULTATI EKF vs RTS (LINEE CONTINUE) ---
% Assicuriamoci che la finestra venga portata in primo piano
fig = figure('Name', 'Confronto EKF vs RTS Smoother (Linee Continue)', 'Color', 'w', 'Position', [100, 100, 900, 600]);
figure(fig); % Forza MATLAB a mostrare la finestra

% --- GRAFICO 1: Angolo di Pitch (Alpha) ---
subplot(2,1,1);
plot(t_out, x_corr(1,:), 'Color', [0 0.4470 0.7410], 'LineWidth', 2.5); hold on;
% MODIFICA QUI: LineStyle cambiato da '--' a '-'
plot(t_out, x_smooth(1,:), 'Color', [0.8500 0.3250 0.0980], 'LineStyle', '-', 'LineWidth', 1);
title('\textbf{Angolo di Beccheggio (Pitch, $\alpha$)}', 'Interpreter', 'latex', 'FontSize', 14);
legend('EKF (Stima Online)', 'RTS Smoother (Stima Offline)', 'Location', 'best', 'FontSize', 11);
ylabel('Angolo [rad]', 'Interpreter', 'latex');
grid on;

% --- GRAFICO 2: Angolo di Yaw (Beta) ---
subplot(2,1,2);
plot(t_out, x_corr(3,:), 'Color', [0 0.4470 0.7410], 'LineWidth', 2.5); hold on;
% MODIFICA QUI: LineStyle cambiato da '--' a '-'
plot(t_out, x_smooth(3,:), 'Color', [0.8500 0.3250 0.0980], 'LineStyle', '-', 'LineWidth', 1);
title('\textbf{Angolo di Imbardata (Yaw, $\beta$)}', 'Interpreter', 'latex', 'FontSize', 14);
legend('EKF (Stima Online)', 'RTS Smoother (Stima Offline)', 'Location', 'best', 'FontSize', 11);
xlabel('Tempo [s]', 'Interpreter', 'latex'); 
ylabel('Angolo [rad]', 'Interpreter', 'latex');
grid on;