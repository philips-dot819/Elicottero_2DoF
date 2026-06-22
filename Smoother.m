%% --- PASSO ALL'INDIETRO: RAUCH-TUNG-STRIEBEL (RTS) SMOOTHER ---
disp('--- INIZIO ESECUZIONE SCRIPT ---');

% 1. Estrazione e Correzione Forzata delle Dimensioni
x_corr = out.x_hat_out';                    % Trasposta: da 10001x4 a 4x10001
x_pred = reshape(out.x_prior_out, 4, []);   % Reshape forzato: da 4x1x10001 a 4x10001
P_corr = out.P_out;                         % 4x4x10001
P_pred = out.P_prior_out;                   % 4x4x10001
F_mat  = out.F_jacob_out;                   % 4x4x10001
t_out  = out.tout;                          

% NUOVO: Estrazione dello stato reale dal blocco To Workspace
% Nota: assumiamo che il formato sia Array. Se è Timeseries, usa out.x_true_out.Data'
x_true = out.x_true_out';                   % Trasposta: da 10001x4 a 4x10001
x_true(1,:) = wrapToPi(x_true(1,:));
x_true(3,:) = wrapToPi(x_true(3,:));

disp(['Dimensione x_corr: ', num2str(size(x_corr,1)), 'x', num2str(size(x_corr,2))]);
disp(['Dimensione x_true: ', num2str(size(x_true,1)), 'x', num2str(size(x_true,2))]);

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
    
    % Calcolo della "differenza" dello stato
    dx = x_smooth(:, k+1) - x_pred(:, k+1);
    
    % WRAPPING DELLA DIFFERENZA (Solo per le posizioni angolari 1 e 3)
    dx(1) = wrapToPi(dx(1));
    dx(3) = wrapToPi(dx(3));
    % dx(2) e dx(4) relativi alle velocità NON si wrappano
    
    % Aggiornamento dello Stato Lisciato
    x_smooth(:, k) = x_corr(:, k) + C_k * dx;
    
    % Wrapping dello stato finale (Solo per gli angoli)
    x_smooth(1, k) = wrapToPi(x_smooth(1, k));
    x_smooth(3, k) = wrapToPi(x_smooth(3, k));
    
    % Aggiornamento della Covarianza Lisciata
    P_smooth(:, :, k) = P_k_corr + C_k * (P_smooth(:, :, k+1) - P_k1_pred) * C_k';
    
    % Mantenimento della simmetria
    P_smooth(:, :, k) = (P_smooth(:, :, k) + P_smooth(:, :, k)') / 2;
end
disp('RTS Smoothing calcolato. Generazione grafici in corso...');

%% --- VISUALIZZAZIONE RISULTATI: STATO REALE vs EKF vs RTS ---
fig = figure('Name', 'Confronto Stato Reale vs EKF vs RTS Smoother', 'Color', 'w', 'Position', [100, 100, 1200, 800]);

% --- GRAFICO 1: Angolo di Pitch (Alpha) ---
subplot(2,2,1);
plot(t_out, x_true(1,:), 'k', 'LineWidth', 2); hold on;
plot(t_out, x_corr(1,:), 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);
plot(t_out, x_smooth(1,:), 'Color', [0.8500 0.3250 0.0980], 'LineStyle', '-', 'LineWidth', 1.5);
title('$$\mathbf{Pitch\ (\alpha)}$$', 'Interpreter', 'latex', 'FontSize', 12);
legend('Reale', 'EKF (Online)', 'RTS (Offline)', 'Location', 'best');
ylabel('$$\textrm{Angolo}\ [\textrm{rad}]$$', 'Interpreter', 'latex');
grid on;

% --- GRAFICO 2: Velocità di Pitch (Alpha_dot) ---
subplot(2,2,2);
plot(t_out, x_true(2,:), 'k', 'LineWidth', 2); hold on;
plot(t_out, x_corr(2,:), 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);
plot(t_out, x_smooth(2,:), 'Color', [0.8500 0.3250 0.0980], 'LineStyle', '-', 'LineWidth', 1.5);
title('$$\mathbf{Velocit\grave{a}\ Pitch\ (\dot{\alpha})}$$', 'Interpreter', 'latex', 'FontSize', 12);
legend('Reale', 'EKF (Online)', 'RTS (Offline)', 'Location', 'best');
ylabel('$$\textrm{Velocita}\ [\textrm{rad/s}]$$', 'Interpreter', 'latex');
grid on;

% --- GRAFICO 3: Angolo di Yaw (Beta) ---
subplot(2,2,3);
plot(t_out, x_true(3,:), 'k', 'LineWidth', 2); hold on;
plot(t_out, x_corr(3,:), 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);
plot(t_out, x_smooth(3,:), 'Color', [0.8500 0.3250 0.0980], 'LineStyle', '-', 'LineWidth', 1.5);
title('$$\mathbf{Yaw\ (\beta)}$$', 'Interpreter', 'latex', 'FontSize', 12);
legend('Reale', 'EKF (Online)', 'RTS (Offline)', 'Location', 'best');
xlabel('$$\textrm{Tempo}\ [\textrm{s}]$$', 'Interpreter', 'latex'); 
ylabel('$$\textrm{Angolo}\ [\textrm{rad}]$$', 'Interpreter', 'latex');
grid on;

% --- GRAFICO 4: Velocità di Yaw (Beta_dot) ---
subplot(2,2,4);
plot(t_out, x_true(4,:), 'k', 'LineWidth', 2); hold on;
plot(t_out, x_corr(4,:), 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);
plot(t_out, x_smooth(4,:), 'Color', [0.8500 0.3250 0.0980], 'LineStyle', '-', 'LineWidth', 1.5);
title('$$\mathbf{Velocit\grave{a}\ Yaw\ (\dot{\beta})}$$', 'Interpreter', 'latex', 'FontSize', 12);
legend('Reale', 'EKF (Online)', 'RTS (Offline)', 'Location', 'best');
xlabel('$$\textrm{Tempo}\ [\textrm{s}]$$', 'Interpreter', 'latex'); 
ylabel('$$\textrm{Velocita}\ [\textrm{rad/s}]$$', 'Interpreter', 'latex');
grid on;

%% --- NUOVA FINESTRA: ANALISI E CONFRONTO DEGLI ERRORI DI STIMA ---
% 1. Calcolo degli errori di stima (Stato Reale - Stato Stimato)
err_ekf = x_true - x_corr;
err_rts = x_true - x_smooth;

% WRAPPING DELL'ERRORE: Si applica solo agli angoli (stati 1 e 3)
err_ekf(1,:) = wrapToPi(err_ekf(1,:));
err_ekf(3,:) = wrapToPi(err_ekf(3,:));
err_rts(1,:) = wrapToPi(err_rts(1,:));
err_rts(3,:) = wrapToPi(err_rts(3,:));

% CORREZIONE LATEX: Aggiunti i simboli del dollaro ($) per la modalità matematica
stati_nomi = {'Pitch ($\alpha$)', 'Velocit\`a Pitch ($\dot{\alpha}$)', 'Yaw ($\beta$)', 'Velocit\`a Yaw ($\dot{\beta}$)'};
unita_misura = {'[rad]', '[rad/s]', '[rad]', '[rad/s]'};

% Inizializzazione della finestra
fig_err = figure('Name', 'Analisi RMSE: EKF vs RTS', 'Color', 'w', 'Position', [150, 50, 1000, 850]);

for i = 1:4
    % Calcolo dell'RMSE (Root Mean Square Error) per EKF e RTS
    rmse_ekf = sqrt(mean(err_ekf(i,:).^2));
    rmse_rts = sqrt(mean(err_rts(i,:).^2));
    
    % Calcolo del miglioramento percentuale dell'RTS rispetto all'EKF
    miglioramento = ((rmse_ekf - rmse_rts) / rmse_ekf) * 100;
    
    % Generazione dei Subplot
    subplot(2,2,i);
    plot(t_out, err_ekf(i,:), 'Color', [0 0.4470 0.7410], 'LineWidth', 1.2); hold on;
    plot(t_out, err_rts(i,:), 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.2);
    
    % Titoli corretti con sintassi LaTeX
    title(sprintf('\\textbf{Errore di Stima: %s}', stati_nomi{i}), 'Interpreter', 'latex', 'FontSize', 12);
    xlabel('Tempo [s]', 'Interpreter', 'latex');
    ylabel(sprintf('$\\Delta$ %s', unita_misura{i}), 'Interpreter', 'latex');
    grid on;
    
    % Legenda pulita e posizionata in alto a destra
    legend({'Errore EKF', 'Errore RTS'}, 'Location', 'northeast', 'FontSize', 10);
    
    % INSERIMENTO DATI SUL GRAFICO: Box di testo semi-trasparente in basso a sinistra
    testo_stat = sprintf('RMSE EKF: %.4f\nRMSE RTS: %.4f\nMiglioramento RTS: %.1f%%', rmse_ekf, rmse_rts, miglioramento);
    text(0.02, 0.05, testo_stat, 'Units', 'normalized', ...
        'BackgroundColor', [1 1 1 0.85], 'EdgeColor', [0.3 0.3 0.3], ...
        'FontSize', 10, 'Interpreter', 'none', 'VerticalAlignment', 'bottom');
end