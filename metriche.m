%% ============================================================
%  CONFRONTO METRICHE STIMATORI - EKF, PF, RTS
%  usando direttamente gli errori salvati da Simulink

%  Metriche:
%       - RMSE classico
%       - P95 errore assoluto 
%       - errore massimo
%       - tempo di assestamento con errore istantaneo
%% ============================================================


close all;

disp('--- Analisi metriche stimatori ---');

%% ============================================================
%  1. Tempo comune della simulazione
%% ============================================================

t = out.tout(:)';
dt = median(diff(t));

fprintf('Tempo simulazione: %.3f s\n', t(end));
fprintf('Campioni su tempo comune: %d\n', length(t));
fprintf('Passo medio dt: %.6f s\n\n', dt);

%% ============================================================
%  2. Caricamento errori EKF e PF
%% ============================================================

err_ekf = [];
err_pf  = [];

if hasVariable(out, 'e_ekf')
    err_ekf = loadErrorFromOut(out, 'e_ekf', t);
    fprintf('Errore EKF caricato da Simulink: e_ekf\n');
else
    warning('Errore EKF non trovato. Controlla che il To Workspace si chiami e_ekf.');
end

if hasVariable(out, 'e_pf')
    err_pf = loadErrorFromOut(out, 'e_pf', t);
    fprintf('Errore PF caricato da Simulink: e_pf\n');
else
    warning('Errore PF non trovato. Controlla che il To Workspace si chiami e_pf.');
end

%% ============================================================
%  3. Errore RTS calcolato offline
%% ============================================================

err_rts = [];

if exist('x_smooth', 'var') && hasVariable(out, 'x_true_out')

    x_true = loadStateFromOut(out, 'x_true_out', t);
    x_true = wrapAngles(x_true);

    x_rts = alignStateToReferenceTime(x_smooth, [], t);
    x_rts = wrapAngles(x_rts);

    err_rts = computeStateError(x_true, x_rts);

    fprintf('Errore RTS calcolato offline da x_true_out e x_smooth\n');

else
    warning('RTS non disponibile. Servono x_smooth e out.x_true_out.');
end

fprintf('\nDimensioni errori caricati:\n');

if ~isempty(err_ekf)
    fprintf('  err_ekf: 4 x %d\n', size(err_ekf, 2));
end

if ~isempty(err_pf)
    fprintf('  err_pf : 4 x %d\n', size(err_pf, 2));
end

if ~isempty(err_rts)
    fprintf('  err_rts: 4 x %d\n', size(err_rts, 2));
end

fprintf('\n');

%% ============================================================
%  4. Parametri delle metriche
%% ============================================================

tol = tol; %dai parameters

% alpha      -> 0.05 rad circa 2.9 gradi
% alpha_dot  -> 0.30 rad/s
% beta       -> 0.05 rad circa 2.9 gradi
% beta_dot   -> 0.30 rad/s

min_check_time = 0.10;
idx_min_check = find(t >= min_check_time, 1, 'first');

if isempty(idx_min_check)
    idx_min_check = 1;
end

%% ============================================================
%  5. Lista stimatori
%% ============================================================

estimators = struct('name', {}, 'type', {}, 'err', {});

if ~isempty(err_ekf)
    estimators(end+1).name = "EKF";
    estimators(end).type   = "Online";
    estimators(end).err    = err_ekf;
end

if ~isempty(err_pf)
    estimators(end+1).name = "PF";
    estimators(end).type   = "Online";
    estimators(end).err    = err_pf;
end

if ~isempty(err_rts)
    estimators(end+1).name = "RTS";
    estimators(end).type   = "Offline";
    estimators(end).err    = err_rts;
end

if isempty(estimators)
    error('Nessuno stimatore disponibile. Controlla e_ekf, e_pf e x_smooth.');
end

%% ============================================================
%  6. Calcolo metriche
%% ============================================================

for s = 1:length(estimators)

    name = estimators(s).name;
    err  = estimators(s).err;

    checkCompatibleSize(err, t, name);

    err(1,:) = wrapToPi(err(1,:));
    err(3,:) = wrapToPi(err(3,:));

    estimators(s).err = err;

    t_settle_state = nan(4,1);

    for i = 1:4
        t_settle_state(i) = findSettlingTime( ...
            err(i,:), ...
            t, ...
            tol(i), ...
            idx_min_check);
    end

    if all(~isnan(t_settle_state))
        t_settle_global = max(t_settle_state);
    else
        t_settle_global = NaN;
    end

    if isnan(t_settle_global)
        t_start_metrics = t(1);
        idx_metrics = true(size(t));
    else
        t_start_metrics = t_settle_global;
        idx_metrics = t >= t_start_metrics;
    end

    [RMSE, P95, EMAX] = computeMetrics(err, idx_metrics);

    estimators(s).RMSE = RMSE;
    estimators(s).P95 = P95;
    estimators(s).EMAX = EMAX;
    estimators(s).t_settle_global = t_settle_global;
    estimators(s).t_settle_state = t_settle_state;
    estimators(s).t_start_metrics = t_start_metrics;

end

%% ============================================================
%  7. Tabelle per report
%     Tabella 1 = parametri principali
%     Tabella 2 = metriche stimatori
%% ============================================================

%% ------------------------------
%  7.1 Tabella parametri principali
%% ------------------------------

parameter_names = [
    "Seed random fissato"
    "Seed random"
    "Parametri fisici plant"
    "Condizione iniziale reale"
    "Rumore sensori"
    "Deviazione standard camera"
    "Deviazione standard accelerometro"
    "Outlier"
    "Probabilita outlier"
    "Ampiezza outlier camera"
    "Ampiezza outlier accelerometro"
    "Matrice rumore di processo EKF Q_EKF"
    "Matrice rumore di misura EKF R_EKF"
    "Stato iniziale ipotizzato EKF x0_EKF"
    "Covarianza iniziale EKF P0_EKF"
    "Soglia Mahalanobis EKF"
    "Numero particelle PF N_PF"
    "Distribuzione iniziale particelle PF"
    "Particelle iniziali PF X0_PF"
    "Matrice rumore di processo PF Q_PF"
    "Matrice rumore di misura PF R_PF"
    "Soglia resampling PF"
    "Soglie assestamento tol"
];

parameter_area = [
    "Configurazione"
    "Configurazione"
    "Modello"
    "Modello"
    "Sensori"
    "Sensori"
    "Sensori"
    "Outlier"
    "Outlier"
    "Outlier"
    "Outlier"
    "EKF"
    "EKF"
    "EKF"
    "EKF"
    "EKF"
    "PF"
    "PF"
    "PF"
    "PF"
    "PF"
    "PF"
    "Metriche"
];

parameter_choice = [
    getValueIfExists('flag_fixed_seed')
    getValueIfExists('rng_seed')
    getValueIfExists('plant_param_case')
    getValueIfExists('x0_case')
    getValueIfExists('sensor_noise_case')
    "—"
    "—"
    getValueIfExists('flag_use_outliers')
    "—"
    "—"
    "—"
    getValueIfExists('Q_EKF_case')
    "—"
    getValueIfExists('x0_EKF_case')
    getValueIfExists('P0_EKF_case')
    "—"
    getValueIfExists('N_PF_case')
    getValueIfExists('X0_PF_case')
    "—"
    getValueIfExists('Q_PF_case')
    "—"
    "—"
    "—"
];

parameter_values = [
    getValueIfExists('flag_fixed_seed')
    getValueIfExists('rng_seed')
    getPlantParamDescription()
    getValueIfExists('x0')
    getSensorNoiseDescription()
    getValueIfExists('std_dev_cam')
    getValueIfExists('std_dev_acc')
    getOutlierDescription()
    getValueIfExists('prob_outlier')
    getValueIfExists('amp_outlier_cam')
    getValueIfExists('amp_outlier_acc')
    getValueIfExists('Q_EKF')
    getValueIfExists('R_EKF')
    getValueIfExists('x0_EKF')
    getValueIfExists('P0_EKF')
    getValueIfExists('threshold_maha')
    getValueIfExists('N_PF')
    "-"
    getValueIfExists('X0_PF')
    getValueIfExists('Q_PF')
    getValueIfExists('R_PF')
    getValueIfExists('resampling_th_PF')
    formatValueForReport(tol, 'tol')
];

parameter_table = table( ...
    parameter_names, ...
    parameter_area, ...
    parameter_choice, ...
    parameter_values, ...
    'VariableNames', {'Parametro', 'Ambito', 'Scelta', 'Valore'} ...
);

disp(' ');
disp('========== TABELLA PARAMETRI ==========');
disp(parameter_table);

writeTextTable(parameter_table, 'tabella_parametri_stimatori.txt');

disp(' ');
disp('Tabella parametri salvata come: tabella_parametri_stimatori.txt');
disp(' ');
%% ------------------------------
%  7.2 Tabella metriche
%% ------------------------------

metric_names = [
    "RMSE alpha [rad]"
    "RMSE alpha dot [rad/s]"
    "RMSE beta [rad]"
    "RMSE beta dot [rad/s]"
    "Tempo assestamento globale [s]"
    "Tempo assestamento alpha [s]"
    "Tempo assestamento alpha dot [s]"
    "Tempo assestamento beta [s]"
    "Tempo assestamento beta dot [s]"
    "P95 alpha [rad]"
    "P95 alpha dot [rad/s]"
    "P95 beta [rad]"
    "P95 beta dot [rad/s]"
    "Errore max alpha [rad]"
    "Errore max alpha dot [rad/s]"
    "Errore max beta [rad]"
    "Errore max beta dot [rad/s]"
    "Metriche calcolate da [s]"
];

report_table = table(metric_names, 'VariableNames', {'Misura'});

for s = 1:length(estimators)

    estimator_name = char(estimators(s).name);

    values = {
        estimators(s).RMSE(1)
        estimators(s).RMSE(2)
        estimators(s).RMSE(3)
        estimators(s).RMSE(4)
        estimators(s).t_settle_global
        estimators(s).t_settle_state(1)
        estimators(s).t_settle_state(2)
        estimators(s).t_settle_state(3)
        estimators(s).t_settle_state(4)
        estimators(s).P95(1)
        estimators(s).P95(2)
        estimators(s).P95(3)
        estimators(s).P95(4)
        estimators(s).EMAX(1)
        estimators(s).EMAX(2)
        estimators(s).EMAX(3)
        estimators(s).EMAX(4)
        estimators(s).t_start_metrics
    };

    report_table.(estimator_name) = valuesToString(values);

end

disp(' ');
disp('========== TABELLA METRICHE PER REPORT ==========');
disp(report_table);

writeTextTable(report_table, 'tabella_metriche_stimatori_report.txt');

disp(' ');
disp('Tabella metriche salvata come: tabella_metriche_stimatori_report.txt');

%% ============================================================
%  8. Grafici
%% ============================================================

stateNames = {'alpha', 'alpha dot', 'beta', 'beta dot'};
stateUnits = {'rad', 'rad/s', 'rad', 'rad/s'};
colors = lines(length(estimators));

plotRawErrors(t, estimators, tol, stateNames, stateUnits, colors);
plotRMSEBars(estimators);
plotSettlingBars(estimators);

%% ============================================================
%  FUNZIONI LOCALI
%% ============================================================

function tf = hasVariable(out, varName)

    vars = out.who;
    tf = any(strcmp(vars, varName));

end

function E = loadErrorFromOut(out, varName, t_ref)

    sig = out.get(varName);

    if isa(sig, 'timeseries')
        E_raw = sig.Data;
        t_sig = sig.Time(:)';
    else
        E_raw = sig;
        t_sig = t_ref(:)';
    end

    E = force4xN(E_raw);

    if length(t_sig) ~= size(E,2)
        t_sig = linspace(t_ref(1), t_ref(end), size(E,2));
    end

    E = alignErrorToReferenceTime(E, t_sig, t_ref);

    E(1,:) = wrapToPi(E(1,:));
    E(3,:) = wrapToPi(E(3,:));

end

function X = loadStateFromOut(out, varName, t_ref)

    sig = out.get(varName);

    if isa(sig, 'timeseries')
        X_raw = sig.Data;
        t_sig = sig.Time(:)';
    else
        X_raw = sig;
        t_sig = t_ref(:)';
    end

    X = alignStateToReferenceTime(X_raw, t_sig, t_ref);

end

function X = force4xN(X_raw)

    X = squeeze(X_raw);

    if ndims(X) > 2
        error('Segnale con dimensioni non gestite: %s', mat2str(size(X_raw)));
    end

    if size(X,1) == 4
        return;
    elseif size(X,2) == 4
        X = X';
    else
        error('Formato segnale non riconosciuto. Dimensioni trovate: %s', mat2str(size(X)));
    end

end

function E_aligned = alignErrorToReferenceTime(E, t_sig, t_ref)

    E = force4xN(E);

    t_sig = t_sig(:)';
    t_ref = t_ref(:)';

    if size(E,2) == length(t_ref) && length(t_sig) == length(t_ref)
        E_aligned = E;
        return;
    end

    fprintf('Allineamento temporale errore: %d campioni -> %d campioni\n', ...
        size(E,2), length(t_ref));

    E_aligned = zeros(4, length(t_ref));

    E_aligned(1,:) = wrapToPi(interp1(t_sig, E(1,:), t_ref, 'linear', 'extrap'));
    E_aligned(2,:) = interp1(t_sig, E(2,:), t_ref, 'linear', 'extrap');
    E_aligned(3,:) = wrapToPi(interp1(t_sig, E(3,:), t_ref, 'linear', 'extrap'));
    E_aligned(4,:) = interp1(t_sig, E(4,:), t_ref, 'linear', 'extrap');

end

function X_aligned = alignStateToReferenceTime(X, t_sig, t_ref)

    X = force4xN(X);

    if isempty(t_sig)
        t_sig = linspace(t_ref(1), t_ref(end), size(X,2));
    end

    t_sig = t_sig(:)';
    t_ref = t_ref(:)';

    if size(X,2) == length(t_ref) && length(t_sig) == length(t_ref)
        X_aligned = X;
        return;
    end

    fprintf('Allineamento temporale stato: %d campioni -> %d campioni\n', ...
        size(X,2), length(t_ref));

    X_aligned = zeros(4, length(t_ref));

    alpha_unwrapped = unwrap(X(1,:));
    beta_unwrapped  = unwrap(X(3,:));

    X_aligned(1,:) = wrapToPi(interp1(t_sig, alpha_unwrapped, t_ref, 'linear', 'extrap'));
    X_aligned(3,:) = wrapToPi(interp1(t_sig, beta_unwrapped,  t_ref, 'linear', 'extrap'));

    X_aligned(2,:) = interp1(t_sig, X(2,:), t_ref, 'linear', 'extrap');
    X_aligned(4,:) = interp1(t_sig, X(4,:), t_ref, 'linear', 'extrap');

end

function X = wrapAngles(X)

    X(1,:) = wrapToPi(X(1,:));
    X(3,:) = wrapToPi(X(3,:));

end

function err = computeStateError(x_true, x_hat)

    err = x_true - x_hat;

    err(1,:) = wrapToPi(err(1,:));
    err(3,:) = wrapToPi(err(3,:));

end

function checkCompatibleSize(err, t, name)

    if size(err,2) ~= length(t)
        error('Dimensioni incompatibili per %s: err = %s, t = %s', ...
            name, mat2str(size(err)), mat2str(size(t)));
    end

end

function t_settle = findSettlingTime(e, t, tol, idx_start)

    e_abs = abs(e);
    outside = e_abs >= tol;

    outside(1:idx_start-1) = false;

    last_outside = find(outside, 1, 'last');

    if isempty(last_outside)
        t_settle = t(idx_start);
    elseif last_outside < length(t)
        t_settle = t(last_outside + 1);
    else
        t_settle = NaN;
    end

end

function [RMSE, P95, EMAX] = computeMetrics(err, idx_metrics)

    RMSE = zeros(4,1);
    P95  = zeros(4,1);
    EMAX = zeros(4,1);

    for i = 1:4

        e = err(i, idx_metrics);

        RMSE(i) = sqrt(mean(e.^2));
        P95(i)  = prctile(abs(e), 95);
        EMAX(i) = max(abs(e));

    end

end

function values_str = valuesToString(values)

    values_str = strings(length(values), 1);

    for k = 1:length(values)
        values_str(k) = formatValueForReport(values{k}, "");
    end

end

function value_str = getValueIfExists(varName)

    if evalin('base', ['exist(''', varName, ''', ''var'')'])
        value = evalin('base', varName);
        value_str = formatValueForReport(value, varName);
    else
        value_str = "non trovato";
    end

end

function value_str = formatValueForReport(value, varName)

    if nargin < 2
        varName = "";
    end

    if isnumeric(value)

        if isempty(value)
            value_str = "[]";
            return;
        end

        % Caso scalare
        if isscalar(value)
            value_str = formatScalarForReport(value);
            return;
        end

        % Caso matrice diagonale: la stampo come diag([...])
        if ismatrix(value) && size(value,1) == size(value,2) && isDiagonalMatrix(value)

            d = diag(value);
            elems = strings(1, length(d));

            for k = 1:length(d)
                elems(k) = formatScalarForReport(d(k));
            end

            diag_str = "diag([" + join(elems, ", ") + "])";

            if strlength(string(varName)) > 0
                value_str = string(varName) + " = " + diag_str;
            else
                value_str = diag_str;
            end

            return;
        end

        % Caso matrice grande non diagonale: non la stampo tutta
        if ismatrix(value) && numel(value) > 25
            value_str = "matrice " + string(size(value,1)) + "x" + string(size(value,2));
            return;
        end

        % Caso vettore
        if isvector(value)

            elems = strings(1, numel(value));

            for k = 1:numel(value)
                elems(k) = formatScalarForReport(value(k));
            end

            if iscolumn(value)
                value_str = "[" + join(elems, "; ") + "]";
            else
                value_str = "[" + join(elems, " ") + "]";
            end

            return;
        end

        % Caso matrice piccola generica
        if ismatrix(value)

            rows = strings(size(value,1), 1);

            for r = 1:size(value,1)

                elems = strings(1, size(value,2));

                for c = 1:size(value,2)
                    elems(c) = formatScalarForReport(value(r,c));
                end

                rows(r) = join(elems, " ");

            end

            value_str = "[" + join(rows, "; ") + "]";
            return;
        end

        value_str = "array " + mat2str(size(value));

    elseif ischar(value) || isstring(value)

        value_str = string(value);

    elseif islogical(value)

        value_str = string(value);

    else

        value_str = "<" + string(class(value)) + ">";

    end

end

function plotRawErrors(t, estimators, tol, stateNames, stateUnits, colors)

    figure('Name','Errori di stima', ...
           'Color','w', ...
           'Position',[100 80 1300 800]);

    tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

    for i = 1:4

        nexttile;
        hold on;
        grid on;
        box on;

        all_errors = [];

        for s = 1:length(estimators)

            e = estimators(s).err(i,:);
            all_errors = [all_errors, e];

            plot(t, e, ...
                'LineWidth', 1.1, ...
                'Color', colors(s,:), ...
                'DisplayName', char(estimators(s).name));
        end

        yline(tol(i),  '--k', 'LineWidth', 0.8, 'HandleVisibility','off');
        yline(-tol(i), '--k', 'LineWidth', 0.8, 'HandleVisibility','off');

        for s = 1:length(estimators)

            ts = estimators(s).t_settle_global;

            if ~isnan(ts)
                xline(ts, ':', ...
                    'Color', colors(s,:), ...
                    'LineWidth', 1.1, ...
                    'HandleVisibility','off');
            end
        end

        ymax = prctile(abs(all_errors), 99);
        ymax = max([ymax, tol(i)*1.3, 1e-6]);
        ylim([-1.15*ymax, 1.15*ymax]);

        title(['Errore ', stateNames{i}], 'Interpreter','none');
        xlabel('Tempo [s]');
        ylabel(['Errore [', stateUnits{i}, ']']);
        legend('Location','best');
    end

    sgtitle('Errori di stima con soglie e tempi di assestamento', ...
            'FontWeight','bold');

end

function plotRMSEBars(estimators)

    figure('Name','RMSE', ...
           'Color','w', ...
           'Position',[150 120 1150 500]);

    rmse_matrix = zeros(length(estimators), 4);

    for s = 1:length(estimators)
        rmse_matrix(s,:) = estimators(s).RMSE(:)';
    end

    bar(rmse_matrix);
    grid on;
    box on;

    names = arrayfun(@(e) char(e.name), estimators, 'UniformOutput', false);

    xticks(1:length(estimators));
    xticklabels(names);

    ylabel('RMSE');
    title('Confronto RMSE');

    legend({'alpha [rad]', ...
            'alpha dot [rad/s]', ...
            'beta [rad]', ...
            'beta dot [rad/s]'}, ...
            'Location','best');

end

function plotSettlingBars(estimators)

    figure('Name','Tempo di assestamento globale', ...
           'Color','w', ...
           'Position',[200 150 800 450]);

    values = zeros(length(estimators), 1);

    for s = 1:length(estimators)
        values(s) = estimators(s).t_settle_global;
    end

    bar(values);
    grid on;
    box on;

    names = arrayfun(@(e) char(e.name), estimators, 'UniformOutput', false);

    xticks(1:length(estimators));
    xticklabels(names);

    ylabel('Tempo di assestamento globale [s]');
    title('Tempo di assestamento globale');

    for s = 1:length(estimators)

        if ~isnan(values(s))
            text(s, values(s), ...
                sprintf(' %.3f s', values(s)), ...
                'VerticalAlignment','bottom', ...
                'HorizontalAlignment','center');
        end

    end

end

function writeTextTable(T, filename)

fid = fopen(filename, 'w');

if fid == -1
    error('Impossibile creare il file: %s', filename);
end

cleaner = onCleanup(@() fclose(fid));

% Scrive intestazione
varNames = T.Properties.VariableNames;

for j = 1:length(varNames)
    fprintf(fid, '%s', varNames{j});

    if j < length(varNames)
        fprintf(fid, '\t');
    else
        fprintf(fid, '\n');
    end
end

% Scrive righe
for i = 1:height(T)

    for j = 1:width(T)

        value = T{i,j};

        if iscell(value)
            value = value{1};
        end

        if isstring(value)
            value = char(value);
        elseif isnumeric(value)
            value = sprintf('%.2e', value);
        elseif ismissing(value)
            value = 'NaN';
        else
            value = char(string(value));
        end

        fprintf(fid, '%s', value);

        if j < width(T)
            fprintf(fid, '\t');
        else
            fprintf(fid, '\n');
        end
    end
end

end

function tf = isDiagonalMatrix(A)

off_diag = A - diag(diag(A));
tol = 1e-14 * max(1, norm(A, 'fro'));

tf = norm(off_diag, 'fro') <= tol;

end


function s = formatScalarForReport(x)

if isnan(x)
    s = "NaN";
    return;
end

if x == 0
    s = "0";
    return;
end

% Notazione scientifica compatta:
% 1.00e-06 -> 1e-6
% 1.00e-02 -> 1e-2
% 3.00e-01 -> 3e-1

raw = sprintf('%.3e', x);

parts = split(string(raw), "e");

mantissa = parts(1);
exponent = str2double(parts(2));

% Tolgo zeri inutili dalla mantissa
mantissa = regexprep(mantissa, '0+$', '');
mantissa = regexprep(mantissa, '\.$', '');

s = mantissa + "e" + string(exponent);

end


function description = getSensorNoiseDescription()

if evalin('base', 'exist(''sensor_noise_case'', ''var'')')
    sensor_case = evalin('base', 'sensor_noise_case');
else
    description = "non trovato";
    return;
end

switch string(sensor_case)

    case "basso"
        description = "basso: std_dev_cam = 5e-3 m, std_dev_acc = 1e-1 m/s^2";

    case "nominale"
        description = "nominale: std_dev_cam = 5e-2 m, std_dev_acc = 5e-1 m/s^2";

    case "alto"
        description = "alto: std_dev_cam = 1e-1 m, std_dev_acc = 1e0 m/s^2";

    otherwise
        description = "caso sensori non riconosciuto";

end

end

function description = getOutlierDescription()

if evalin('base', 'exist(''flag_use_outliers'', ''var'')')
    flag_out = evalin('base', 'flag_use_outliers');
else
    description = "non trovato";
    return;
end

if flag_out
    description = "attivi: prob = 1e-2, amp_cam = 1e0 m, amp_acc = 1e1 m/s^2";
else
    description = "disattivati: prob = 0, amp_cam = 0, amp_acc = 0";
end

end

function description = getPFInitialDistributionDescription()

if evalin('base', 'exist(''X0_PF_case'', ''var'')')
    X0_case = evalin('base', 'X0_PF_case');
else
    description = "non trovato";
    return;
end

switch string(X0_case)

    case "ampia"
        description = "ampia: alpha in [-pi/2, pi/2], alpha_dot in [-1,1], beta in [-pi,pi], beta_dot in [-1,1]";

    case "stretta_attorno_x0"
        description = "stretta attorno a x0: gaussiane su alpha, alpha_dot, beta, beta_dot";

    otherwise
        description = "caso X0_PF non riconosciuto";

end

end

function description = getPlantParamDescription()

if evalin('base', 'exist(''plant_param_case'', ''var'')')
    plant_case = evalin('base', 'plant_param_case');
else
    description = "non trovato";
    return;
end

switch string(plant_case)

    case "nominali"
        description = "nominali: helicopter_params_unc = helicopter_params";

    case "incerti"
        description = "incerti: J 5%, m 1%, l 1%, c 7%, eps 7%, g invariata";

    otherwise
        description = "caso plant non riconosciuto";

end

end