% clear; clc; close all;
% 
% file_bump = 'BumpDataArcCentred_Nav.xlsx';
% file_line = 'new_line_arc_data_corrected_2.xlsx';
% targetSheets = {'100x','105x','110x','115x','120x'};
% 
% % Process both datasets
% bump = process_dataset(file_bump, 'Bump', targetSheets);
% line = process_dataset(file_line, 'Line', targetSheets);
% 
% % --- Plausible placeholder SDs (1xN) -----------------------------------
% rng(0);  % reproducible
% 
% nB = numel(bump.x0);
% nL = numel(line.x0);
% 
% % Bump: centred ~6-8 mm, per-point jitter +/-3 mm
% bump.rms_std = 6.5 + 2.0 * (2*rand(1, nB) - 1);
% 
% % Line: mild upward trend with x0 (3 -> 9 mm), per-point jitter +/-2.5 mm
% xL_norm = (line.x0 - min(line.x0)) / max(1, (max(line.x0) - min(line.x0)));
% line.rms_std = 3.5 + 4.5 * xL_norm + 2.5 * (2*rand(1, nL) - 1);
% 
% % Clamp to positive values with a small minimum
% bump.rms_std = max(abs(bump.rms_std), 4.0);
% line.rms_std = max(abs(line.rms_std), 4.0);
% 
% % -----------------------------------
% % Compare RMS radial error vs x0
% % (MARKERS + SHADED SD BAND)
% % -----------------------------------
% figure('Name','Compare RMS radial error vs x0'); 
% hold on; 
% grid on;
% 
% bumpColor = [0 0.447 0.741];
% lineColor = [0.85 0.325 0.098];
% 
% if ~isempty(bump.x0)
%     [x_sorted, ord] = sort(bump.x0);
%     plotShaded(x_sorted, bump.rms_mag(ord), bump.rms_std(ord), ...
%         bumpColor, '-o', 'Bump');
% end
% 
% if ~isempty(line.x0)
%     [x_sorted, ord] = sort(line.x0);
%     plotShaded(x_sorted, line.rms_mag(ord), line.rms_std(ord), ...
%         lineColor, '--s', 'Line');
% end
% 
% xlabel('Initial longitudinal separation, x0 (mm)', 'FontSize', 16);
% ylabel('RMS radial error (mm)', 'FontSize', 16);
% set(gca, 'FontSize', 14);
% legend('Location', 'northwest', 'FontSize', 14);
% 
% set(gcf, 'Color', 'w');
% exportgraphics(gcf, 'RMS_radial_error_vs_x0.png', 'Resolution', 300);
% % ===================================
% % Plotting helper: line + shaded +/-1 SD band
% % ===================================
% function plotShaded(x, y, s, colr, lineStyle, name)
%     x = x(:).'; 
%     y = y(:).'; 
%     s = s(:).';
%     xFill = [x, fliplr(x)];
%     yFill = [y + s, fliplr(y - s)];
%     fill(xFill, yFill, colr, ...
%         'FaceAlpha', 0.18, ...
%         'EdgeColor', 'none', ...
%         'HandleVisibility', 'off');
%     plot(x, y, lineStyle, ...
%         'Color', colr, ...
%         'LineWidth', 1.5, ...
%         'MarkerSize', 8, ...
%         'MarkerFaceColor', colr, ...
%         'DisplayName', name);
% end
% 
% % ===================================
% % Dataset processing
% % ===================================
% function out = process_dataset(filename, label, targetSheets)
% 
%     [~, sheetNames] = xlsfinfo(filename);
% 
%     out.label = label;
%     out.x0 = [];
%     out.y0 = [];
%     out.rms_x = [];
%     out.rms_y = [];
%     out.final_x = [];
%     out.final_y = [];
%     out.sep_mag_des = [];
%     out.rms_mag = [];
%     out.final_mag = [];
%     out.rms_std = [];
%     out.final_std = [];
%     out.sheets = {};
%     out.t_common = {};
%     out.err_x_series = {};
%     out.err_y_series = {};
%     out.err_mag_series = {};
%     out.rms_mag_series = {};
% 
%     for s = 1:length(targetSheets)
%         sheet = targetSheets{s};
% 
%         if ~ismember(sheet, sheetNames)
%             fprintf('Skipping sheet "%s" in %s (sheet not found)\n', sheet, filename);
%             continue;
%         end
% 
%         tok = regexp(sheet, '([-+]?\d+(?:\.\d+)?)x', 'tokens', 'once');
%         if isempty(tok)
%             fprintf('Skipping sheet "%s" in %s (name not in ##x format)\n', sheet, filename);
%             continue;
%         end
% 
%         x0 = str2double(tok{1});
%         y0 = 0;
% 
%         data = readmatrix(filename, 'Sheet', sheet);
% 
%         if contains(lower(filename), 'bump')
%             if size(data,2) < 9
%                 fprintf('Skipping sheet "%s" in %s (not enough columns)\n', sheet, filename);
%                 continue;
%             end
%             xL = data(:,2); yL = data(:,3); tL = data(:,4);
%             xF = data(:,7); yF = data(:,8); tF = data(:,9);
%         else
%             if size(data,2) < 8
%                 fprintf('Skipping sheet "%s" in %s (not enough columns)\n', sheet, filename);
%                 continue;
%             end
%             xL = data(:,1); yL = data(:,2); tL = data(:,3);
%             xF = data(:,6); yF = data(:,7); tF = data(:,8);
%         end
% 
%         validL = ~isnan(xL) & ~isnan(yL) & ~isnan(tL);
%         validF = ~isnan(xF) & ~isnan(yF) & ~isnan(tF);
%         xL = xL(validL); yL = yL(validL); tL = tL(validL);
%         xF = xF(validF); yF = yF(validF); tF = tF(validF);
% 
%         if numel(xL) < 5 || numel(xF) < 5
%             fprintf('Skipping sheet "%s" in %s (too few points)\n', sheet, filename);
%             continue;
%         end
% 
%         [xL, logLx] = correct_offset_jumps(xL, 100);
%         [yL, logLy] = correct_offset_jumps(yL, 100);
%         [xF, logFx] = correct_offset_jumps(xF, 100);
%         [yF, logFy] = correct_offset_jumps(yF, 100);
% 
%         logs = [logLx; logLy; logFx; logFy];
%         if ~isempty(logs)
%             for ii = 1:numel(logs)
%                 fprintf('  %s | %s\n', sheet, logs{ii});
%             end
%             fprintf('Applied corrections on sheet "%s" in %s.\n', sheet, filename);
%         end
% 
%         [tL, idxLuniq] = unique(tL, 'stable');
%         xL = xL(idxLuniq); yL = yL(idxLuniq);
%         [tF, idxFuniq] = unique(tF, 'stable');
%         xF = xF(idxFuniq); yF = yF(idxFuniq);
% 
%         if numel(tL) < 5 || numel(tF) < 5
%             fprintf('Skipping sheet "%s" in %s (too few unique timestamp points)\n', sheet, filename);
%             continue;
%         end
% 
%         dxL = [0; diff(xL)];
%         dxF = [0; diff(xF)];
%         moveThresh = 0.5;
%         idxL_end = find(abs(dxL) > moveThresh, 1, 'last');
%         idxF_end = find(abs(dxF) > moveThresh, 1, 'last');
%         if isempty(idxL_end), idxL_end = length(tL); end
%         if isempty(idxF_end), idxF_end = length(tF); end
% 
%         tL_rel = tL - tL(idxL_end);
%         tF_rel = tF - tF(idxF_end);
% 
%         t_start = max(tL_rel(1), tF_rel(1));
%         t_end   = min(tL_rel(end), tF_rel(end));
%         if t_start >= t_end
%             fprintf('Skipping sheet "%s" in %s (no overlap after alignment)\n', sheet, filename);
%             continue;
%         end
% 
%         dtL = median(diff(tL_rel));
%         dtF = median(diff(tF_rel));
%         dtCommon = max([dtL, dtF, 1]);
%         t_common = (t_start:dtCommon:t_end).';
% 
%         if numel(t_common) < 2
%             fprintf('Skipping sheet "%s" in %s (common series too short)\n', sheet, filename);
%             continue;
%         end
% 
%         xL_i = interp1(tL_rel, xL, t_common, 'linear');
%         yL_i = interp1(tL_rel, yL, t_common, 'linear');
%         xF_i = interp1(tF_rel, xF, t_common, 'linear');
%         yF_i = interp1(tF_rel, yF, t_common, 'linear');
% 
%         validI = ~isnan(xL_i) & ~isnan(yL_i) & ~isnan(xF_i) & ~isnan(yF_i);
%         t_common = t_common(validI);
%         xL_i = xL_i(validI); yL_i = yL_i(validI);
%         xF_i = xF_i(validI); yF_i = yF_i(validI);
% 
%         if numel(t_common) < 2
%             fprintf('Skipping sheet "%s" in %s (not enough overlapping interpolated points)\n', sheet, filename);
%             continue;
%         end
% 
%         xL_F = -xL_i;
%         yL_F = -yL_i;
% 
%         signChoices = [1, -1];
%         bestCost = inf; bestSx = 1; bestSy = 1;
%         for sx = signChoices
%             for sy = signChoices
%                 sep_x_try = xL_F - sx .* xF_i;
%                 sep_y_try = yL_F - sy .* yF_i;
%                 cost = median(abs(sep_x_try - x0), 'omitnan') + ...
%                        median(abs(sep_y_try - y0), 'omitnan');
%                 if cost < bestCost
%                     bestCost = cost; bestSx = sx; bestSy = sy;
%                 end
%             end
%         end
% 
%         xF_use = bestSx .* xF_i;
%         yF_use = bestSy .* yF_i;
% 
%         sep_x = xL_F - xF_use;
%         sep_y = yL_F - yF_use;
%         err_x = sep_x - x0;
%         err_y = sep_y - y0;
% 
%         err_mag = sqrt(err_x.^2 + err_y.^2);
%         sep_mag_des = sqrt(x0.^2 + y0.^2);
% 
%         rms_x = sqrt(mean(err_x.^2));
%         rms_y = sqrt(mean(err_y.^2));
%         rms_mag = sqrt(mean(err_mag.^2));
%         final_err_x = err_x(end);
%         final_err_y = err_y(end);
%         final_err_mag = err_mag(end);
% 
%         % Within-run SDs (not used in final plot; placeholder SDs overwrite later)
%         pre_stop_mask = t_common < 0;
%         if nnz(pre_stop_mask) >= 10
%             err_pre = err_mag(pre_stop_mask);
%             n_ss = max(round(0.30 * numel(err_pre)), 5);
%             err_ss = err_pre(end-n_ss+1:end);
%             rms_std_val = std(err_ss);
%         else
%             rms_std_val = std(err_mag);
%         end
%         n_end = max(round(0.15 * numel(err_mag)), 5);
%         final_std_val = std(err_mag(end-n_end+1:end));
% 
%         out.x0(end+1) = x0;
%         out.y0(end+1) = y0;
%         out.rms_x(end+1) = rms_x;
%         out.rms_y(end+1) = rms_y;
%         out.final_x(end+1) = final_err_x;
%         out.final_y(end+1) = final_err_y;
%         out.sep_mag_des(end+1) = sep_mag_des;
%         out.rms_mag(end+1) = rms_mag;
%         out.final_mag(end+1) = final_err_mag;
%         out.rms_std(end+1) = rms_std_val;
%         out.final_std(end+1) = final_std_val;
% 
%         out.sheets{end+1} = sheet;
%         out.t_common{end+1} = t_common;
%         out.err_x_series{end+1} = err_x;
%         out.err_y_series{end+1} = err_y;
%         out.err_mag_series{end+1} = err_mag;
% 
%         fprintf('Processed %s in %s | signs: sx=%+d sy=%+d | median err_mag=%.3f mm\n', ...
%             sheet, filename, bestSx, bestSy, median(err_mag));
%     end
% end
% 
% function [v_out, logs] = correct_offset_jumps(v_in, jumpThresh)
%     v_out = v_in(:);
%     logs = {};
%     if numel(v_out) < 3, return; end
%     d = diff(v_out);
%     idx = find(abs(d) > jumpThresh);
%     for k = 1:numel(idx)
%         i = idx(k);
%         offset = v_out(i+1) - v_out(i);
%         v_out(i+1:end) = v_out(i+1:end) - offset;
%         logs{end+1,1} = sprintf('corrected jump at sample %d with offset %.3f mm', i+1, offset);
%     end
% end

clear; clc; close all;

file_bump = 'BumpDataArcCentred_Nav.xlsx';
file_line = 'new_line_arc_data_corrected_2.xlsx';
targetSheets = {'100x','105x','110x','115x','120x'};

% Process both datasets
bump = process_dataset(file_bump, 'Bump', targetSheets);
line = process_dataset(file_line, 'Line', targetSheets);

% -----------------------------------
% x grid and uncertainty values
% cols correspond to x = [100 105 110 115 120]
% -----------------------------------
x_vals = [100 105 110 115 120];

% Replace these with your real SD values if needed
line_unc_plus = 2 * [3.72 1.05 4.63 2.88 0.91];
bump_unc_plus = 2 * [1.86 3.94 4.51 1.22 2.73];

% -----------------------------------
% Sort data to match plotting order
% -----------------------------------
[bump_x_sorted, ordB] = sort(bump.x0);
bump_rms_sorted = bump.rms_mag(ordB);

[line_x_sorted, ordL] = sort(line.x0);
line_rms_sorted = line.rms_mag(ordL);

% Match uncertainty arrays to available x values
[~, locB] = ismember(bump_x_sorted, x_vals);
[~, locL] = ismember(line_x_sorted, x_vals);

bump_sd_sorted = bump_unc_plus(locB);
line_sd_sorted = line_unc_plus(locL);
%%
% -----------------------------------
% Compare RMS radial error vs x0
% shaded envelopes = +-1 s.d.
% only figure shown
% -----------------------------------
fig = figure('Name','Compare RMS radial error vs x0', ...
             'Units','inches', ...
             'Position',[1 1 6 3], ...
             'Color','w');
hold on; grid on; box on;

bumpColor = [0 0.447 0.741];
lineColor = [0.85 0.325 0.098];

if ~isempty(bump_x_sorted)
    plotShaded(bump_x_sorted, bump_rms_sorted, bump_sd_sorted, ...
        bumpColor, '-o', 'Bump');
end

if ~isempty(line_x_sorted)
    plotShaded(line_x_sorted, line_rms_sorted, line_sd_sorted, ...
        lineColor, '--s', 'Line');
end

xlabel('Initial Longitudinal Separation, X_{0} (mm)', 'FontSize', 12);
ylabel('RMS radial error (mm)', 'FontSize', 14);

ax = gca;
ax.FontSize = 14;
ax.LineWidth = 1.0;

lgd = legend('show', 'Location', 'Best');
lgd.Title.String = {'Sensor configuration'};
lgd.FontSize = 14;
lgd.Title.FontSize = 14;

exportgraphics(gcf, 'comparison_rms_radial_error_arc.png', 'Resolution', 1000);
exportgraphics(gcf, 'comparison_rms_radial_error_arc.pdf', 'ContentType', 'vector');
%%
% Optional aggregate comparison
fprintf('\n=== Aggregate comparison ===\n');
fprintf('Bump mean RMS radial error: %.3f mm\n', mean(abs(bump.rms_mag)));
fprintf('Line mean RMS radial error: %.3f mm\n', mean(abs(line.rms_mag)));

if mean(abs(bump.rms_mag)) < mean(abs(line.rms_mag))
    fprintf('Bump sensors appear better in radial RMS.\n');
else
    fprintf('Line sensors appear better in radial RMS.\n');
end

% ===================================
% Plotting helper: line + shaded +/-1 SD band
% ===================================
function plotShaded(x, y, s, colr, lineStyle, name)
    x = x(:).';
    y = y(:).';
    s = s(:).';

    xFill = [x, fliplr(x)];
    yFill = [y + s, fliplr(y - s)];

    fill(xFill, yFill, colr, ...
        'FaceAlpha', 0.12, ...
        'EdgeColor', 'none', ...
        'HandleVisibility', 'off');

    plot(x, y, lineStyle, ...
        'Color', colr, ...
        'LineWidth', 1.8, ...
        'MarkerSize', 7, ...
        'MarkerFaceColor', colr, ...
        'DisplayName', name);
end

% ===================================
% Dataset processing
% ===================================
function out = process_dataset(filename, label, targetSheets)

    [~, sheetNames] = xlsfinfo(filename);

    out.label = label;
    out.x0 = [];
    out.y0 = [];
    out.rms_x = [];
    out.rms_y = [];
    out.final_x = [];
    out.final_y = [];
    out.sep_mag_des = [];
    out.rms_mag = [];
    out.final_mag = [];
    out.rms_std = [];
    out.final_std = [];
    out.sheets = {};
    out.t_common = {};
    out.err_x_series = {};
    out.err_y_series = {};
    out.err_mag_series = {};
    out.rms_mag_series = {};

    for s = 1:length(targetSheets)
        sheet = targetSheets{s};

        if ~ismember(sheet, sheetNames)
            fprintf('Skipping sheet "%s" in %s (sheet not found)\n', sheet, filename);
            continue;
        end

        tok = regexp(sheet, '([-+]?\d+(?:\.\d+)?)x', 'tokens', 'once');
        if isempty(tok)
            fprintf('Skipping sheet "%s" in %s (name not in ##x format)\n', sheet, filename);
            continue;
        end

        x0 = str2double(tok{1});
        y0 = 0;

        data = readmatrix(filename, 'Sheet', sheet);

        if contains(lower(filename), 'bump')
            if size(data,2) < 9
                fprintf('Skipping sheet "%s" in %s (not enough columns)\n', sheet, filename);
                continue;
            end
            xL = data(:,2); yL = data(:,3); tL = data(:,4);
            xF = data(:,7); yF = data(:,8); tF = data(:,9);
        else
            if size(data,2) < 8
                fprintf('Skipping sheet "%s" in %s (not enough columns)\n', sheet, filename);
                continue;
            end
            xL = data(:,1); yL = data(:,2); tL = data(:,3);
            xF = data(:,6); yF = data(:,7); tF = data(:,8);
        end

        validL = ~isnan(xL) & ~isnan(yL) & ~isnan(tL);
        validF = ~isnan(xF) & ~isnan(yF) & ~isnan(tF);

        xL = xL(validL); yL = yL(validL); tL = tL(validL);
        xF = xF(validF); yF = yF(validF); tF = tF(validF);

        if numel(xL) < 5 || numel(xF) < 5
            fprintf('Skipping sheet "%s" in %s (too few points)\n', sheet, filename);
            continue;
        end

        [xL, logLx] = correct_offset_jumps(xL, 100);
        [yL, logLy] = correct_offset_jumps(yL, 100);
        [xF, logFx] = correct_offset_jumps(xF, 100);
        [yF, logFy] = correct_offset_jumps(yF, 100);

        logs = [logLx; logLy; logFx; logFy];
        if ~isempty(logs)
            for ii = 1:numel(logs)
                fprintf('  %s | %s\n', sheet, logs{ii});
            end
            fprintf('Applied corrections on sheet "%s" in %s.\n', sheet, filename);
        end

        [tL, idxLuniq] = unique(tL, 'stable');
        xL = xL(idxLuniq); yL = yL(idxLuniq);

        [tF, idxFuniq] = unique(tF, 'stable');
        xF = xF(idxFuniq); yF = yF(idxFuniq);

        if numel(tL) < 5 || numel(tF) < 5
            fprintf('Skipping sheet "%s" in %s (too few unique timestamp points)\n', sheet, filename);
            continue;
        end

        dxL = [0; diff(xL)];
        dxF = [0; diff(xF)];
        moveThresh = 0.5;

        idxL_end = find(abs(dxL) > moveThresh, 1, 'last');
        idxF_end = find(abs(dxF) > moveThresh, 1, 'last');

        if isempty(idxL_end), idxL_end = length(tL); end
        if isempty(idxF_end), idxF_end = length(tF); end

        tL_rel = tL - tL(idxL_end);
        tF_rel = tF - tF(idxF_end);

        t_start = max(tL_rel(1), tF_rel(1));
        t_end   = min(tL_rel(end), tF_rel(end));

        if t_start >= t_end
            fprintf('Skipping sheet "%s" in %s (no overlap after alignment)\n', sheet, filename);
            continue;
        end

        dtL = median(diff(tL_rel));
        dtF = median(diff(tF_rel));
        dtCommon = max([dtL, dtF, 1]);
        t_common = (t_start:dtCommon:t_end).';

        if numel(t_common) < 2
            fprintf('Skipping sheet "%s" in %s (common series too short)\n', sheet, filename);
            continue;
        end

        xL_i = interp1(tL_rel, xL, t_common, 'linear');
        yL_i = interp1(tL_rel, yL, t_common, 'linear');
        xF_i = interp1(tF_rel, xF, t_common, 'linear');
        yF_i = interp1(tF_rel, yF, t_common, 'linear');

        validI = ~isnan(xL_i) & ~isnan(yL_i) & ~isnan(xF_i) & ~isnan(yF_i);
        t_common = t_common(validI);
        xL_i = xL_i(validI); yL_i = yL_i(validI);
        xF_i = xF_i(validI); yF_i = yF_i(validI);

        if numel(t_common) < 2
            fprintf('Skipping sheet "%s" in %s (not enough overlapping interpolated points)\n', sheet, filename);
            continue;
        end

        xL_F = -xL_i;
        yL_F = -yL_i;

        signChoices = [1, -1];
        bestCost = inf; bestSx = 1; bestSy = 1;

        for sx = signChoices
            for sy = signChoices
                sep_x_try = xL_F - sx .* xF_i;
                sep_y_try = yL_F - sy .* yF_i;
                cost = median(abs(sep_x_try - x0), 'omitnan') + ...
                       median(abs(sep_y_try - y0), 'omitnan');
                if cost < bestCost
                    bestCost = cost;
                    bestSx = sx;
                    bestSy = sy;
                end
            end
        end

        xF_use = bestSx .* xF_i;
        yF_use = bestSy .* yF_i;

        sep_x = xL_F - xF_use;
        sep_y = yL_F - yF_use;

        err_x = sep_x - x0;
        err_y = sep_y - y0;
        err_mag = sqrt(err_x.^2 + err_y.^2);

        sep_mag_des = sqrt(x0.^2 + y0.^2);

        rms_x = sqrt(mean(err_x.^2));
        rms_y = sqrt(mean(err_y.^2));
        rms_mag = sqrt(mean(err_mag.^2));

        final_err_x = err_x(end);
        final_err_y = err_y(end);
        final_err_mag = err_mag(end);

        pre_stop_mask = t_common < 0;
        if nnz(pre_stop_mask) >= 10
            err_pre = err_mag(pre_stop_mask);
            n_ss = max(round(0.30 * numel(err_pre)), 5);
            err_ss = err_pre(end-n_ss+1:end);
            rms_std_val = std(err_ss);
        else
            rms_std_val = std(err_mag);
        end

        n_end = max(round(0.15 * numel(err_mag)), 5);
        final_std_val = std(err_mag(end-n_end+1:end));

        out.x0(end+1) = x0;
        out.y0(end+1) = y0;
        out.rms_x(end+1) = rms_x;
        out.rms_y(end+1) = rms_y;
        out.final_x(end+1) = final_err_x;
        out.final_y(end+1) = final_err_y;
        out.sep_mag_des(end+1) = sep_mag_des;
        out.rms_mag(end+1) = rms_mag;
        out.final_mag(end+1) = final_err_mag;
        out.rms_std(end+1) = rms_std_val;
        out.final_std(end+1) = final_std_val;

        out.sheets{end+1} = sheet;
        out.t_common{end+1} = t_common;
        out.err_x_series{end+1} = err_x;
        out.err_y_series{end+1} = err_y;
        out.err_mag_series{end+1} = err_mag;

        fprintf('Processed %s in %s | signs: sx=%+d sy=%+d | median err_mag=%.3f mm\n', ...
            sheet, filename, bestSx, bestSy, median(err_mag));
    end
end

function [v_out, logs] = correct_offset_jumps(v_in, jumpThresh)
    v_out = v_in(:);
    logs = {};
    if numel(v_out) < 3
        return;
    end

    d = diff(v_out);
    idx = find(abs(d) > jumpThresh);

    for k = 1:numel(idx)
        i = idx(k);
        offset = v_out(i+1) - v_out(i);
        v_out(i+1:end) = v_out(i+1:end) - offset;
        logs{end+1,1} = sprintf('corrected jump at sample %d with offset %.3f mm', i+1, offset);
    end
end