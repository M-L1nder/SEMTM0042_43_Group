%% =========================================================================
%  ROBOT LEADER-FOLLOWER PATH ERROR ANALYSIS
%  =========================================================================
%  Description:
%    Loads all sheets from an Excel workbook containing Leader and Follower
%    robot position data. Sheet names follow the convention:
%      "x<X_OFFSET>_y<Y_OFFSET>"   e.g. "x100_y0", "x105_y-30"
%
%    For each sheet the script:
%      1. Parses the x and y offsets from the sheet name
%      2. Extracts Follower (x_f, y_f) and Leader (x_l, y_l) positions
%      3. Interpolates Leader positions onto the Follower sample grid
%      4. Computes lateral (cross-track) path error at every sample
%      5. Computes RMSE and peak error per run
%
%    Then produces several diagnostic / presentation-quality plots:
%      Plot 1 – Lateral error vs. distance travelled  (per x-offset group)
%      Plot 2 – Lateral RMSE vs. initial y-offset     (per x-offset group)
%      Plot 3 – Peak lateral error vs. y-offset        (per x-offset group)
%      Plot 4 – 3-D surface / scatter: distance × y-offset × error
%      Plot 5 – Heatmap of RMSE across all (x-offset, y-offset) combos
%      Plot 6 – Error convergence: final error vs. y-offset
%
%  Data format expected inside each sheet:
%    Row 1  : "Follower"  [blank cols]  "Leader"
%    Row 2  : i  x_mm  y_mm  ts  [blank]  i  x_mm  y_mm  ts
%    Row 3+ : data rows
%    Follower columns: A(1), B(2), C(3)  →  index, x_mm, y_mm
%    Leader  columns: F(6), G(7), H(8)  →  index, x_mm, y_mm
%    (1-based column indexing as read by readcell)
%
%  Usage:
%    1. Set EXCEL_FILE to the path of your .xlsx file.
%    2. Run the script – all plots will be generated automatically.
%    3. Add more tabs (e.g. "x105_y0", "x110_y-30") and re-run;
%       the script discovers all sheets automatically.
%  =========================================================================

clear; clc; close all;

%% ── USER SETTINGS ────────────────────────────────────────────────────────
EXCEL_FILE   = 'LINE_STRAIGHT_TESTS.xlsx';   % <-- update path if needed
INTERP_METHOD = 'linear';                     % interpolation for leader→follower grid
DIST_UNIT    = 'mm';                          % label for distance axis

% Colour-map for offset curves
CMAP = turbo;

%% ── LOAD SHEET NAMES ─────────────────────────────────────────────────────
[~, sheets] = xlsfinfo(EXCEL_FILE);
fprintf('Found %d sheet(s) in "%s"\n', numel(sheets), EXCEL_FILE);

%% ── PARSE SHEET NAMES ────────────────────────────────────────────────────
% Expected pattern: x<number>_y<number>  (number may be negative)
pattern = '^x(-?\d+)_y(-?\d+)$';

x_offsets_all = [];
y_offsets_all = [];
valid_sheets  = {};

for s = 1:numel(sheets)
    tok = regexp(sheets{s}, pattern, 'tokens');
    if ~isempty(tok)
        x_offsets_all(end+1) = str2double(tok{1}{1}); %#ok<SAGROW>
        y_offsets_all(end+1) = str2double(tok{1}{2}); %#ok<SAGROW>
        valid_sheets{end+1}  = sheets{s};             %#ok<SAGROW>
    else
        fprintf('  Skipping sheet "%s" – name does not match pattern.\n', sheets{s});
    end
end

unique_x = unique(x_offsets_all);
unique_y = sort(unique(y_offsets_all));

fprintf('  x-offsets detected : %s\n', mat2str(unique_x));
fprintf('  y-offsets detected : %s\n', mat2str(unique_y));
fprintf('  Valid sheets       : %d\n\n', numel(valid_sheets));

%% ── READ AND PROCESS ALL SHEETS ──────────────────────────────────────────
results = struct();   % will hold per-sheet results

for s = 1:numel(valid_sheets)
    sname   = valid_sheets{s};
    x_off   = x_offsets_all(s);
    y_off   = y_offsets_all(s);

    % --- read raw cell array (skip header rows) ---------------------------
    raw = readcell(EXCEL_FILE, 'Sheet', sname, 'NumHeaderLines', 2);

    % Columns (1-based): Follower = 1,2,3 (i,x,y), Leader = 6,7,8 (i,x,y)
    % Extract numeric data, skip rows that are entirely missing
    f_x_raw = cellfun(@(c) double_or_nan(c), raw(:,2));
    f_y_raw = cellfun(@(c) double_or_nan(c), raw(:,3));
    l_x_raw = cellfun(@(c) double_or_nan(c), raw(:,7));
    l_y_raw = cellfun(@(c) double_or_nan(c), raw(:,8));

    % Keep only rows where follower has valid data
    f_valid = ~isnan(f_x_raw) & ~isnan(f_y_raw);
    l_valid = ~isnan(l_x_raw) & ~isnan(l_y_raw);

    f_x = f_x_raw(f_valid);
    f_y = f_y_raw(f_valid);
    l_x = l_x_raw(l_valid);
    l_y = l_y_raw(l_valid);

    if numel(f_x) < 2 || numel(l_x) < 2
        fprintf('  WARNING: insufficient data in sheet "%s", skipping.\n', sname);
        continue
    end

    % --- compute arc-length (distance travelled) along follower path ------
    d_f = cumulative_distance(f_x, f_y);

    % --- compute arc-length along leader path -----------------------------
    d_l = cumulative_distance(l_x, l_y);

    % --- interpolate leader onto follower's distance grid -----------------
    % interp1 requires unique sample points. Repeated cumulative-distance
    % values can occur when successive samples have zero movement, so keep
    % the last sample at each repeated distance.
    [d_l_u, iu_l] = unique(d_l, 'last');
    l_x_u = l_x(iu_l);
    l_y_u = l_y(iu_l);

    % Clamp follower distances to leader range to avoid extrapolation
    d_f_clamped = min(max(d_f, d_l_u(1)), d_l_u(end));

    l_x_interp = interp1(d_l_u, l_x_u, d_f_clamped, INTERP_METHOD, 'extrap');
    l_y_interp = interp1(d_l_u, l_y_u, d_f_clamped, INTERP_METHOD, 'extrap');

    % --- lateral error (perpendicular to intended straight-line path) -----
    % The intended path is a straight line. We estimate its direction from
    % the leader trajectory (principal axis).  For a nominal straight run
    % this is approximately along x; we derive it properly via PCA so the
    % method generalises to any heading.
    pts = [l_x, l_y];
    mu  = mean(pts,1);
    C   = cov(pts);
    [V, ~] = eig(C);
    along_vec = V(:,2)';   % eigenvector of largest eigenvalue = path direction
    lat_vec   = [-along_vec(2), along_vec(1)];  % perpendicular

    % Lateral error = projection of (follower - leader) onto lat_vec
    delta_x = f_x - l_x_interp;
    delta_y = f_y - l_y_interp;
    lat_err = delta_x * lat_vec(1) + delta_y * lat_vec(2);

    % --- performance metrics ----------------------------------------------
    rmse_lat  = sqrt(mean(lat_err.^2));
    peak_lat  = max(abs(lat_err));
    final_lat = lat_err(end);

    % --- store results ----------------------------------------------------
    key = matlab.lang.makeValidName(sname);
    results.(key).sheet     = sname;
    results.(key).x_off     = x_off;
    results.(key).y_off     = y_off;
    results.(key).dist      = d_f;
    results.(key).lat_err   = lat_err;
    results.(key).rmse      = rmse_lat;
    results.(key).peak      = peak_lat;
    results.(key).final_err = final_lat;

    fprintf('  [%s]  RMSE = %6.3f mm  |  Peak = %6.3f mm  |  Final = %6.3f mm\n', ...
            sname, rmse_lat, peak_lat, final_lat);
end

%% ── COLLECT ARRAYS FOR PLOTTING ──────────────────────────────────────────
fnames  = fieldnames(results);
nruns   = numel(fnames);

all_x_off  = zeros(nruns,1);
all_y_off  = zeros(nruns,1);
all_rmse   = zeros(nruns,1);
all_peak   = zeros(nruns,1);
all_final  = zeros(nruns,1);

for k = 1:nruns
    r = results.(fnames{k});
    all_x_off(k)  = r.x_off;
    all_y_off(k)  = r.y_off;
    all_rmse(k)   = r.rmse;
    all_peak(k)   = r.peak;
    all_final(k)  = r.final_err;
end

[sorted_y, iy] = sort(all_y_off);

%% ========================================================================
%  PLOT 1 – Lateral Error vs. Distance Travelled (one subplot per x-offset)
%% ========================================================================
fig1 = figure('Name','Lateral Error vs. Distance','Color','w','Position',[50 50 1200 800]);

n_xoff  = numel(unique_x);
cols_sf = ceil(sqrt(n_xoff));
rows_sf = ceil(n_xoff / cols_sf);

for xi = 1:n_xoff
    xo       = unique_x(xi);
    ax       = subplot(rows_sf, cols_sf, xi);
    hold(ax,'on'); grid(ax,'on'); box(ax,'on');

    % gather all runs for this x-offset, sorted by y_off
    idx_x    = find(all_x_off == xo);
    [~, si]  = sort(all_y_off(idx_x));
    idx_x    = idx_x(si);
    n_curves = numel(idx_x);
    cmap_sel = interp1(linspace(0,1,256), CMAP, linspace(0,1,n_curves));

    for ci = 1:n_curves
        r   = results.(fnames{idx_x(ci)});
        plot(ax, r.dist, r.lat_err, 'Color', cmap_sel(ci,:), 'LineWidth', 1.5, ...
             'DisplayName', sprintf('y = %+d mm', r.y_off));
    end

    yline(ax, 0, 'k--', 'LineWidth', 0.8, 'HandleVisibility','off');
    xlabel(ax, sprintf('Distance Travelled (%s)', DIST_UNIT), 'FontSize', 10);
    ylabel(ax, sprintf('Lateral Error (%s)', DIST_UNIT), 'FontSize', 10);
    title(ax, sprintf('x_{offset} = %d mm', xo), 'FontWeight','bold', 'FontSize',11);
    legend(ax, 'Location','best', 'FontSize',7, 'NumColumns', 2);
    set(ax, 'FontSize', 9);
end

sgtitle(fig1, 'Lateral Tracking Error vs. Distance Travelled', ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ========================================================================
%  PLOT 2 – Lateral RMSE vs. Initial y-Offset  (all x-offsets overlaid)
%% ========================================================================
fig2 = figure('Name','RMSE vs. y-Offset','Color','w','Position',[100 100 900 550]);
ax2  = axes(fig2); hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');

markers = {'o','s','^','d','v','p','h','*'};

for xi = 1:n_xoff
    xo    = unique_x(xi);
    idx_x = find(all_x_off == xo);
    [ys_sort, si] = sort(all_y_off(idx_x));
    rs_sort       = all_rmse(idx_x(si));

    plot(ax2, ys_sort, rs_sort, ...
         ['-' markers{mod(xi-1,numel(markers))+1}], ...
         'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor','auto', ...
         'DisplayName', sprintf('x_{offset} = %d mm', xo));
end

xlabel(ax2, 'Initial Lateral Offset y_0 (mm)', 'FontSize', 12);
ylabel(ax2, 'Lateral RMSE (mm)', 'FontSize', 12);
title(ax2, 'Lateral RMSE vs. Initial y-Offset', 'FontWeight','bold', 'FontSize',14);
legend(ax2, 'Location','best', 'FontSize', 10);
set(ax2, 'FontSize', 11);
xline(ax2, 0, 'k--', 'LineWidth', 1, 'HandleVisibility','off');

%% ========================================================================
%  PLOT 3 – Peak Absolute Error vs. y-Offset
%% ========================================================================
fig3 = figure('Name','Peak Error vs. y-Offset','Color','w','Position',[150 150 900 550]);
ax3  = axes(fig3); hold(ax3,'on'); grid(ax3,'on'); box(ax3,'on');

for xi = 1:n_xoff
    xo    = unique_x(xi);
    idx_x = find(all_x_off == xo);
    [ys_sort, si] = sort(all_y_off(idx_x));
    pk_sort       = all_peak(idx_x(si));

    plot(ax3, ys_sort, pk_sort, ...
         ['-' markers{mod(xi-1,numel(markers))+1}], ...
         'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor','auto', ...
         'DisplayName', sprintf('x_{offset} = %d mm', xo));
end

xlabel(ax3, 'Initial Lateral Offset y_0 (mm)', 'FontSize', 12);
ylabel(ax3, 'Peak |Lateral Error| (mm)', 'FontSize', 12);
title(ax3, 'Peak Lateral Error vs. Initial y-Offset', 'FontWeight','bold', 'FontSize',14);
legend(ax3, 'Location','best', 'FontSize', 10);
set(ax3, 'FontSize', 11);
xline(ax3, 0, 'k--', 'LineWidth', 1, 'HandleVisibility','off');

%% ========================================================================
%  PLOT 4 – 3-D Scatter / Surface: Distance × y-Offset × Lateral Error
%  (for a single x-offset; if multiple x-offsets exist, one figure per x)
%% ========================================================================
for xi = 1:n_xoff
    xo    = unique_x(xi);
    idx_x = find(all_x_off == xo);
    if numel(idx_x) < 2, continue; end
    [~, si] = sort(all_y_off(idx_x));
    idx_x   = idx_x(si);

    fig4 = figure('Name', sprintf('3D Error Map x=%d',xo), 'Color','w', ...
                  'Position',[200 200 1000 650]);
    ax4  = axes(fig4); hold(ax4,'on');

    n_curves  = numel(idx_x);
    cmap_sel4 = interp1(linspace(0,1,256), CMAP, linspace(0,1,n_curves));

    % Normalise each run's distance to [0,1] so all runs align despite
    % slightly different total path lengths, then use actual distances for
    % the 3-D axis label.
    all_dist_mat = [];
    all_yoff_mat = [];
    all_lerr_mat = [];

    for ci = 1:n_curves
        r    = results.(fnames{idx_x(ci)});
        nd   = numel(r.dist);
        yoff_vec = repmat(r.y_off, nd, 1);
        all_dist_mat = [all_dist_mat; r.dist(:)];     %#ok<AGROW>
        all_yoff_mat = [all_yoff_mat; yoff_vec];      %#ok<AGROW>
        all_lerr_mat = [all_lerr_mat; r.lat_err(:)];  %#ok<AGROW>
    end

    % Scatter coloured by |error|
    sc = scatter3(ax4, all_dist_mat, all_yoff_mat, all_lerr_mat, ...
                  12, all_lerr_mat, 'filled', 'MarkerFaceAlpha', 0.7);
    colormap(ax4, turbo);
    cb = colorbar(ax4);
    cb.Label.String = 'Lateral Error (mm)';
    cb.FontSize = 10;

    % Draw zero-error plane
    xlims = xlim(ax4);
    ylims = ylim(ax4);
    patch(ax4, [xlims(1) xlims(2) xlims(2) xlims(1)], ...
               [ylims(1) ylims(1) ylims(2) ylims(2)], ...
               [0 0 0 0], 'k', 'FaceAlpha', 0.05, 'EdgeColor','none');

    xlabel(ax4, sprintf('Distance Travelled (%s)', DIST_UNIT), 'FontSize',11);
    ylabel(ax4, 'Initial y-Offset (mm)', 'FontSize',11);
    zlabel(ax4, 'Lateral Error (mm)', 'FontSize',11);
    title(ax4, sprintf('3-D Lateral Error Map  —  x_{offset} = %d mm', xo), ...
          'FontWeight','bold','FontSize',13);
    view(ax4, -40, 25);
    grid(ax4,'on');
    set(ax4,'FontSize',10, 'BoxStyle','full');
end

%% ========================================================================
%  PLOT 5 – Heatmap: RMSE across (x-offset  ×  y-offset)
%  (only meaningful when multiple x-offsets are present)
%% ========================================================================
if n_xoff > 1
    % Build RMSE matrix  [n_yoff × n_xoff]
    n_yoff = numel(unique_y);
    rmse_mat = NaN(n_yoff, n_xoff);
    for k = 1:nruns
        xi_idx = find(unique_x == all_x_off(k));
        yi_idx = find(unique_y == all_y_off(k));
        if ~isempty(xi_idx) && ~isempty(yi_idx)
            rmse_mat(yi_idx, xi_idx) = all_rmse(k);
        end
    end

    fig5 = figure('Name','RMSE Heatmap','Color','w','Position',[250 250 800 600]);
    ax5  = axes(fig5);
    imagesc(ax5, unique_x, unique_y, rmse_mat);
    colormap(ax5, hot);
    cb5 = colorbar(ax5);
    cb5.Label.String = 'Lateral RMSE (mm)';
    cb5.FontSize = 11;
    xlabel(ax5, 'x-Offset (mm)', 'FontSize',12);
    ylabel(ax5, 'y-Offset (mm)', 'FontSize',12);
    title(ax5, 'Heatmap of Lateral RMSE across Offset Grid', ...
          'FontWeight','bold','FontSize',14);
    set(ax5,'YDir','normal','FontSize',11, ...
            'XTick', unique_x, 'YTick', unique_y);
    % Overlay RMSE values
    for xi = 1:n_xoff
        for yi = 1:n_yoff
            if ~isnan(rmse_mat(yi,xi))
                text(ax5, unique_x(xi), unique_y(yi), ...
                     sprintf('%.2f', rmse_mat(yi,xi)), ...
                     'HorizontalAlignment','center','FontSize',8, ...
                     'Color','w','FontWeight','bold');
            end
        end
    end
end

%% ========================================================================
%  PLOT 6 – Final (Residual) Lateral Error vs. y-Offset
%  Shows how well the follower converges by end of run
%% ========================================================================
fig6 = figure('Name','Residual Error vs. y-Offset','Color','w','Position',[300 300 900 550]);
ax6  = axes(fig6); hold(ax6,'on'); grid(ax6,'on'); box(ax6,'on');

for xi = 1:n_xoff
    xo    = unique_x(xi);
    idx_x = find(all_x_off == xo);
    [ys_sort, si] = sort(all_y_off(idx_x));
    fe_sort       = all_final(idx_x(si));

    plot(ax6, ys_sort, fe_sort, ...
         ['-' markers{mod(xi-1,numel(markers))+1}], ...
         'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor','auto', ...
         'DisplayName', sprintf('x_{offset} = %d mm', xo));
end

xlabel(ax6, 'Initial Lateral Offset y_0 (mm)', 'FontSize',12);
ylabel(ax6, 'Residual Lateral Error at End of Run (mm)', 'FontSize',12);
title(ax6, 'Follower Convergence: Final Lateral Error vs. Initial y-Offset', ...
      'FontWeight','bold','FontSize',14);
legend(ax6,'Location','best','FontSize',10);
set(ax6,'FontSize',11);
xline(ax6, 0, 'k--', 'LineWidth', 1, 'HandleVisibility','off');
yline(ax6, 0, 'k--', 'LineWidth', 1, 'HandleVisibility','off');

%% ========================================================================
%  PLOT 7 – Error Envelope: Mean ± Std across all y-offsets per x-offset
%% ========================================================================
fig7 = figure('Name','Error Envelope','Color','w','Position',[350 350 1100 600]);

for xi = 1:n_xoff
    xo    = unique_x(xi);
    idx_x = find(all_x_off == xo);

    % Interpolate all error traces onto a common distance grid
    all_dists = cellfun(@(f) results.(f).dist, fnames(idx_x), 'UniformOutput', false);
    d_min = max(cellfun(@(d) d(1),   all_dists));
    d_max = min(cellfun(@(d) d(end), all_dists));
    d_common = linspace(d_min, d_max, 200)';

    err_mat = NaN(200, numel(idx_x));
    for ci = 1:numel(idx_x)
        r = results.(fnames{idx_x(ci)});

        % Repeated distance samples arise when the robot pauses but is still
        % logged. Collapse duplicates before interpolation so Figure 7 can be
        % formed robustly. Keeping the last sample preserves the most recent
        % lateral error recorded at each travelled distance.
        [d_u, iu] = unique(r.dist, 'last');
        e_u = r.lat_err(iu);

        if numel(d_u) >= 2
            err_mat(:,ci) = interp1(d_u, e_u, d_common, 'linear', NaN);
        end
    end

    mu_err  = mean(err_mat, 2, 'omitnan');
    sd_err  = std(err_mat,  0, 2, 'omitnan');

    ax7 = subplot(1, n_xoff, xi);
    hold(ax7,'on'); grid(ax7,'on'); box(ax7,'on');

    fill([d_common; flipud(d_common)], ...
         [mu_err+sd_err; flipud(mu_err-sd_err)], ...
         [0.6 0.8 1.0], 'FaceAlpha', 0.4, 'EdgeColor','none', ...
         'DisplayName','\pm1\sigma');
    plot(ax7, d_common, mu_err, 'b-', 'LineWidth', 2, 'DisplayName','Mean');
    yline(ax7, 0, 'k--', 'LineWidth', 0.8, 'HandleVisibility','off');

    xlabel(ax7, sprintf('Distance (%s)', DIST_UNIT), 'FontSize',10);
    ylabel(ax7, 'Lateral Error (mm)', 'FontSize',10);
    title(ax7, sprintf('x_{off} = %d mm', xo), 'FontWeight','bold','FontSize',11);
    legend(ax7,'Location','best','FontSize',9);
    set(ax7,'FontSize',9);
end

sgtitle(fig7, 'Lateral Error Envelope (Mean ± 1\sigma across all y-offsets)', ...
        'FontSize',14,'FontWeight','bold');

%% ── SUMMARY TABLE ────────────────────────────────────────────────────────
fprintf('\n%s\n', repmat('=',1,72));
fprintf('  %-18s  %8s  %8s  %8s  %10s\n', ...
        'Sheet', 'x_off', 'y_off', 'RMSE(mm)', 'Peak(mm)');
fprintf('%s\n', repmat('-',1,72));
for k = 1:nruns
    r = results.(fnames{k});
    fprintf('  %-18s  %8d  %8d  %8.3f  %10.3f\n', ...
            r.sheet, r.x_off, r.y_off, r.rmse, r.peak);
end
fprintf('%s\n', repmat('=',1,72));

%% =========================================================================
%  LOCAL HELPER FUNCTIONS
%% =========================================================================

function d = cumulative_distance(x, y)
%CUMULATIVE_DISTANCE  Arc-length along a 2-D path.
    dx = diff(x(:));
    dy = diff(y(:));
    d  = [0; cumsum(sqrt(dx.^2 + dy.^2))];
end

function v = double_or_nan(c)
%DOUBLE_OR_NAN  Convert a cell value to double; return NaN for non-numeric.
    if isnumeric(c)
        v = double(c);
    elseif ischar(c) || isstring(c)
        v = str2double(c);
    else
        v = NaN;
    end
end
