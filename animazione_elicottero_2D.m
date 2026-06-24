%% ============================================================
%  ANIMAZIONE 2D ELICOTTERO 2 DOF
%  Vista laterale + vista dall'alto nella stessa figure
%% ============================================================

if ~exist('out', 'var')
    error('Variabile out non trovata. Prima esegui la simulazione Simulink.');
end

%% ============================================================
%  LETTURA SEGNALI
%% ============================================================

t = out.tout(:)';

x_true_signal = out.get('x_true_out');

if isa(x_true_signal, 'timeseries')
    x_data = x_true_signal.Data;
    t_sig = x_true_signal.Time(:)';
else
    x_data = x_true_signal;
    t_sig = t;
end

x_true = formatStateForAnimation(x_data);

if length(t) ~= size(x_true, 2)
    t = t_sig;
end

alpha = x_true(1, :);
beta  = x_true(3, :);

%% ============================================================
%  OPZIONI ANIMAZIONE
%% ============================================================

step_anim = 1;

flag_save_video = false;
video_filename = 'animazione_elicottero_2DOF_doppia.mp4';

%% ============================================================
%  PARAMETRI GRAFICI
%% ============================================================

L = 0.80;
body_half = 0.045;

mast_h = 0.22;

main_rotor_R_front = 0.28;
tail_rotor_R_front = 0.075;

main_rotor_R_top = 0.16;
tail_rotor_R_top = 0.07;

%% ============================================================
%  FIGURA UNICA CON DUE VISTE
%% ============================================================

fig = figure('Color', 'w');

tl = tiledlayout(fig, 1, 2, ...
    'TileSpacing', 'compact', ...
    'Padding', 'compact');

ax_front = nexttile(tl, 1);
hold(ax_front, 'on');
grid(ax_front, 'on');
axis(ax_front, 'equal');

xlabel(ax_front, 'x [m]');
ylabel(ax_front, 'z [m]');

xlim(ax_front, [-0.78 0.78]);
ylim(ax_front, [-0.65 0.58]);

title(ax_front, 'Vista laterale: pitch \alpha');

set(ax_front, 'FontSize', 11);

ax_top = nexttile(tl, 2);
hold(ax_top, 'on');
grid(ax_top, 'on');
axis(ax_top, 'equal');

xlabel(ax_top, 'x [m]');
ylabel(ax_top, 'y [m]');

xlim(ax_top, [-0.78 0.78]);
ylim(ax_top, [-0.78 0.78]);

title(ax_top, 'Vista dall''alto: yaw \beta');

set(ax_top, 'FontSize', 11);

h_global_title = sgtitle(fig, 'Animazione elicottero 2 DOF');

%% ============================================================
%  VISTA LATERALE - ELEMENTI STATICI
%% ============================================================

% Base e supporto
plot(ax_front, [-0.18 0.18], [-0.58 -0.58], ...
    'Color', [0.35 0.45 0.50], 'LineWidth', 8);

plot(ax_front, [0 0], [-0.58 0], ...
    'Color', [0.45 0.55 0.60], 'LineWidth', 8);

plot(ax_front, 0, 0, 'o', ...
    'MarkerSize', 16, ...
    'MarkerFaceColor', [0.85 0.90 0.92], ...
    'MarkerEdgeColor', [0.15 0.20 0.25], ...
    'LineWidth', 2);

plot(ax_front, 0, 0, 'ko', ...
    'MarkerSize', 6, ...
    'MarkerFaceColor', 'k');

% Riferimento orizzontale
plot(ax_front, [-0.60 0.60], [0 0], '--', ...
    'Color', [0.85 0.15 0.10], ...
    'LineWidth', 1);

% Callout F1 laterale
text(ax_front, 0.50, 0.46, {'$F_1$ [N]', 'rotore principale -- pitch'}, ...
    'Interpreter', 'latex', ...
    'FontSize', 10, ...
    'HorizontalAlignment', 'center', ...
    'BackgroundColor', [1.00 0.93 0.90], ...
    'EdgeColor', [0.65 0.15 0.08], ...
    'Margin', 8);

% Callout F2 laterale
text(ax_front, -0.59, -0.20, {'$F_2$ [N]', 'rotore di coda -- yaw'}, ...
    'Interpreter', 'latex', ...
    'FontSize', 10, ...
    'HorizontalAlignment', 'center', ...
    'BackgroundColor', [1.00 0.94 0.82], ...
    'EdgeColor', [0.60 0.32 0.05], ...
    'Margin', 8);

text(ax_front, 0.31, -0.10, 'asse yaw', ...
    'Color', [0.05 0.28 0.55], ...
    'FontSize', 12);

text(ax_front, 0.46, 0.10, '$mg\sin(\alpha)$', ...
    'Interpreter', 'latex', ...
    'FontSize', 13, ...
    'Color', [0.25 0.30 0.35]);

quiver(ax_front, 0.43, 0.18, 0, -0.22, 0, ...
    'Color', [0.25 0.30 0.35], ...
    'LineWidth', 2.5, ...
    'MaxHeadSize', 0.7);

%% ============================================================
%  VISTA LATERALE - ELEMENTI DINAMICI
%% ============================================================

h_body_front = patch(ax_front, nan, nan, [0.62 0.76 0.82], ...
    'FaceAlpha', 0.65, ...
    'EdgeColor', [0.20 0.30 0.35], ...
    'LineWidth', 1.5);

h_mast_front = plot(ax_front, nan, nan, ...
    'Color', [0.10 0.45 0.15], ...
    'LineWidth', 6);

h_main_rotor_front = patch(ax_front, nan, nan, [0.20 0.65 0.20], ...
    'FaceAlpha', 0.28, ...
    'EdgeColor', [0.10 0.45 0.15], ...
    'LineWidth', 1.5);

h_main_hub_front = plot(ax_front, nan, nan, 'o', ...
    'MarkerSize', 10, ...
    'MarkerFaceColor', [0.05 0.35 0.10], ...
    'MarkerEdgeColor', [0.05 0.25 0.08], ...
    'LineWidth', 1.5);

h_tail_rotor_front = patch(ax_front, nan, nan, [1.00 0.50 0.05], ...
    'FaceAlpha', 0.35, ...
    'EdgeColor', [0.85 0.35 0.02], ...
    'LineWidth', 1.5);

h_tail_hub_front = plot(ax_front, nan, nan, 'o', ...
    'MarkerSize', 8, ...
    'MarkerFaceColor', [1.00 0.35 0.00], ...
    'MarkerEdgeColor', [0.70 0.20 0.00], ...
    'LineWidth', 1.5);

h_skid_front = plot(ax_front, nan, nan, ...
    'Color', [0.20 0.30 0.35], ...
    'LineWidth', 4);

h_alpha_arc_front = plot(ax_front, nan, nan, ...
    'Color', [0.90 0.10 0.08], ...
    'LineWidth', 2);

h_alpha_text_front = text(ax_front, 0, 0, '$\alpha$', ...
    'Interpreter', 'latex', ...
    'FontSize', 14, ...
    'Color', [0.90 0.10 0.08]);

h_beta_text_front = text(ax_front, -0.16, -0.18, '', ...
    'Interpreter', 'latex', ...
    'FontSize', 14, ...
    'Color', [0.05 0.28 0.65]);

%% ============================================================
%  VISTA DALL'ALTO - ELEMENTI STATICI
%% ============================================================

% Fulcro centrale
plot(ax_top, 0, 0, 'o', ...
    'MarkerSize', 18, ...
    'MarkerFaceColor', [0.85 0.90 0.92], ...
    'MarkerEdgeColor', [0.15 0.20 0.25], ...
    'LineWidth', 2);

plot(ax_top, 0, 0, 'ko', ...
    'MarkerSize', 6, ...
    'MarkerFaceColor', 'k');

% Assi riferimento
plot(ax_top, [-0.65 0.65], [0 0], '--', ...
    'Color', [0.80 0.15 0.10], ...
    'LineWidth', 1);

plot(ax_top, [0 0], [-0.65 0.65], '--', ...
    'Color', [0.05 0.28 0.65], ...
    'LineWidth', 1);

text(ax_top, 0.60, 0.04, 'asse x riferimento', ...
    'Color', [0.80 0.15 0.10], ...
    'FontSize', 10);

text(ax_top, 0.03, 0.52, 'asse y', ...
    'Color', [0.05 0.28 0.65], ...
    'FontSize', 10);

% Callout F1 top
text(ax_top, 0.50, 0.58, {'$F_1$ [N]', 'rotore principale'}, ...
    'Interpreter', 'latex', ...
    'FontSize', 10, ...
    'HorizontalAlignment', 'center', ...
    'BackgroundColor', [1.00 0.93 0.90], ...
    'EdgeColor', [0.65 0.15 0.08], ...
    'Margin', 8);

% Callout F2 top
text(ax_top, -0.58, -0.58, {'$F_2$ [N]', 'rotore di coda -- yaw'}, ...
    'Interpreter', 'latex', ...
    'FontSize', 10, ...
    'HorizontalAlignment', 'center', ...
    'BackgroundColor', [1.00 0.94 0.82], ...
    'EdgeColor', [0.60 0.32 0.05], ...
    'Margin', 8);

%% ============================================================
%  VISTA DALL'ALTO - ELEMENTI DINAMICI
%% ============================================================

h_body_top = patch(ax_top, nan, nan, [0.62 0.76 0.82], ...
    'FaceAlpha', 0.65, ...
    'EdgeColor', [0.20 0.30 0.35], ...
    'LineWidth', 1.5);

h_main_rotor_top = patch(ax_top, nan, nan, [0.20 0.65 0.20], ...
    'FaceAlpha', 0.28, ...
    'EdgeColor', [0.10 0.45 0.15], ...
    'LineWidth', 1.5);

h_main_hub_top = plot(ax_top, nan, nan, 'o', ...
    'MarkerSize', 10, ...
    'MarkerFaceColor', [0.05 0.35 0.10], ...
    'MarkerEdgeColor', [0.05 0.25 0.08], ...
    'LineWidth', 1.5);

h_tail_rotor_top = patch(ax_top, nan, nan, [1.00 0.50 0.05], ...
    'FaceAlpha', 0.35, ...
    'EdgeColor', [0.85 0.35 0.02], ...
    'LineWidth', 1.5);

h_tail_hub_top = plot(ax_top, nan, nan, 'o', ...
    'MarkerSize', 8, ...
    'MarkerFaceColor', [1.00 0.35 0.00], ...
    'MarkerEdgeColor', [0.70 0.20 0.00], ...
    'LineWidth', 1.5);

h_beta_arc_top = plot(ax_top, nan, nan, ...
    'Color', [0.05 0.28 0.65], ...
    'LineWidth', 2);

h_beta_text_top = text(ax_top, 0, 0, '$\beta$', ...
    'Interpreter', 'latex', ...
    'FontSize', 14, ...
    'Color', [0.05 0.28 0.65]);

h_alpha_text_top = text(ax_top, -0.65, 0.68, '', ...
    'Interpreter', 'latex', ...
    'FontSize', 13, ...
    'Color', [0.85 0.10 0.08]);

%% ============================================================
%  VIDEO OPZIONALE
%% ============================================================

if flag_save_video
    video_obj = VideoWriter(video_filename, 'MPEG-4');
    video_obj.FrameRate = 25;
    open(video_obj);
end

%% ============================================================
%  LOOP ANIMAZIONE SINCRONIZZATA
%% ============================================================

for k = 1:step_anim:length(t)

    a = alpha(k);
    b = wrapToPiLocal(beta(k));

    %% ------------------------------------------------------------
    %  Aggiornamento vista laterale
    %% ------------------------------------------------------------

    u_front = [cos(a); sin(a)];
    n_front = [-sin(a); cos(a)];

    P0_front = [0; 0];

    P_left_front  = P0_front - (L/2) * u_front;
    P_right_front = P0_front + (L/2) * u_front;

    body_poly_front = [
        P_left_front  + body_half * n_front, ...
        P_right_front + body_half * n_front, ...
        P_right_front - body_half * n_front, ...
        P_left_front  - body_half * n_front
    ];

    set(h_body_front, ...
        'XData', body_poly_front(1, :), ...
        'YData', body_poly_front(2, :));

    P_main_front = P0_front + mast_h * n_front;

    set(h_mast_front, ...
        'XData', [P0_front(1), P_main_front(1)], ...
        'YData', [P0_front(2), P_main_front(2)]);

    [xr_front, zr_front] = rotatedEllipse( ...
        P_main_front(1), P_main_front(2), ...
        main_rotor_R_front, 0.025, a, 100);

    set(h_main_rotor_front, ...
        'XData', xr_front, ...
        'YData', zr_front);

    set(h_main_hub_front, ...
        'XData', P_main_front(1), ...
        'YData', P_main_front(2));

    P_tail_front = P_left_front - 0.03 * u_front;

    [xt_front, zt_front] = rotatedEllipse( ...
        P_tail_front(1), P_tail_front(2), ...
        tail_rotor_R_front, 0.018, a + pi/2, 80);

    set(h_tail_rotor_front, ...
        'XData', xt_front, ...
        'YData', zt_front);

    set(h_tail_hub_front, ...
        'XData', P_tail_front(1), ...
        'YData', P_tail_front(2));

    P_skid_1_front = P0_front - 0.22*u_front - 0.09*n_front;
    P_skid_2_front = P0_front + 0.25*u_front - 0.09*n_front;

    set(h_skid_front, ...
        'XData', [P_skid_1_front(1), P_skid_2_front(1)], ...
        'YData', [P_skid_1_front(2), P_skid_2_front(2)]);

    if abs(a) < 1e-4
        xa_front = nan;
        za_front = nan;
    else
        [xa_front, za_front] = arcPoints(0, 0, 0.13, 0, a, 40);
    end

    set(h_alpha_arc_front, ...
        'XData', xa_front, ...
        'YData', za_front);

    set(h_alpha_text_front, ...
        'Position', [0.16*cos(a/2), 0.16*sin(a/2), 0], ...
        'String', sprintf('$\\alpha = %.1f^\\circ$', rad2deg(a)));

    set(h_beta_text_front, ...
        'String', sprintf('$\\beta = %.1f^\\circ$', rad2deg(b)));

    %% ------------------------------------------------------------
    %  Aggiornamento vista dall'alto
    %% ------------------------------------------------------------

    u_top = [cos(b); sin(b)];
    n_top = [-sin(b); cos(b)];

    L_proj = L * cos(a);

    P0_top = [0; 0];

    P_front_top = P0_top + (L_proj/2) * u_top;
    P_back_top  = P0_top - (L_proj/2) * u_top;

    body_poly_top = [
        P_back_top  + body_half * n_top, ...
        P_front_top + body_half * n_top, ...
        P_front_top - body_half * n_top, ...
        P_back_top  - body_half * n_top
    ];

    set(h_body_top, ...
        'XData', body_poly_top(1, :), ...
        'YData', body_poly_top(2, :));

    [xm_top, ym_top] = circlePoints(0, 0, main_rotor_R_top, 100);

    set(h_main_rotor_top, ...
        'XData', xm_top, ...
        'YData', ym_top);

    set(h_main_hub_top, ...
        'XData', 0, ...
        'YData', 0);

    P_tail_top = P_back_top - 0.04*u_top;

    [xt_top, yt_top] = rotatedEllipse( ...
        P_tail_top(1), P_tail_top(2), ...
        tail_rotor_R_top, 0.018, b + pi/2, 80);

    set(h_tail_rotor_top, ...
        'XData', xt_top, ...
        'YData', yt_top);

    set(h_tail_hub_top, ...
        'XData', P_tail_top(1), ...
        'YData', P_tail_top(2));

    if abs(b) < 1e-4
        xb_top = nan;
        yb_top = nan;
    else
        [xb_top, yb_top] = arcPoints(0, 0, 0.22, 0, b, 50);
    end

    set(h_beta_arc_top, ...
        'XData', xb_top, ...
        'YData', yb_top);

    set(h_beta_text_top, ...
        'Position', [0.27*cos(b/2), 0.27*sin(b/2), 0], ...
        'String', sprintf('$\\beta = %.1f^\\circ$', rad2deg(b)));

    set(h_alpha_text_top, ...
        'String', sprintf('$\\alpha = %.1f^\\circ$', rad2deg(a)));

    %% ------------------------------------------------------------
    %  Titolo globale
    %% ------------------------------------------------------------

    set(h_global_title, ...
        'String', sprintf( ...
        'Elicottero 2 DOF | t = %.2f s | alpha = %.1f deg | beta = %.1f deg', ...
        t(k), rad2deg(a), rad2deg(b)));

    drawnow limitrate;

    if flag_save_video
        writeVideo(video_obj, getframe(fig));
    end

end

if flag_save_video
    close(video_obj);
    fprintf('Video salvato come: %s\n', video_filename);
end

%% ============================================================
%  FUNZIONI LOCALI
%% ============================================================

function x = formatStateForAnimation(x_data)

    x_data = squeeze(x_data);

    if size(x_data, 1) == 4
        x = x_data;
    elseif size(x_data, 2) == 4
        x = x_data';
    else
        error('Formato x_true_out non riconosciuto. Atteso 4xN oppure Nx4.');
    end

end

function [x, y] = circlePoints(cx, cy, r, N)

    th = linspace(0, 2*pi, N);

    x = cx + r*cos(th);
    y = cy + r*sin(th);

end

function [x, z] = rotatedEllipse(cx, cz, a, b, theta, N)

    th = linspace(0, 2*pi, N);

    xe = a*cos(th);
    ze = b*sin(th);

    R = [
        cos(theta), -sin(theta)
        sin(theta),  cos(theta)
    ];

    pts = R * [xe; ze];

    x = cx + pts(1, :);
    z = cz + pts(2, :);

end

function [x, y] = arcPoints(cx, cy, r, th1, th2, N)

    th = linspace(th1, th2, N);

    x = cx + r*cos(th);
    y = cy + r*sin(th);

end

function angle_wrapped = wrapToPiLocal(angle)

    angle_wrapped = mod(angle + pi, 2*pi) - pi;

end