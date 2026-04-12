% Use exactly this style:
%
% x50_y0
% x100_y0
% x100_y20
% x100_y-20

clear; clc; close all;

% filename = 'XY_offset2.xlsx';   % <-- change this

filename = 'XY_OFFSET_line.xlsx';   % <-- change this


[~, sheetNames] = xlsfinfo(filename);

all_x0 = [];
all_y0 = [];
all_rms_x = [];
all_rms_y = [];
all_mae_x = [];
all_mae_y = [];

% Store traces for surface plotting
surf_x0 = [];
surf_y0 = [];
surf_t = {};
surf_errx = {};

figure('Name','Error X vs Time'); hold on; grid on;
xlabel('Time relative to end (ms)');
ylabel('Longitudinal error (mm)');
title('Longitudinal error vs time');

figure('Name','Error Y vs Time'); hold on; grid on;
xlabel('Time relative to end (ms)');
ylabel('Lateral error (mm)');
title('Lateral error vs time');

for s = 1:length(sheetNames)
    sheet = sheetNames{s};

    % Parse sheet name like x50_y-20
    tok = regexp(sheet, 'x([-+]?\d+)_y([-+]?\d+)', 'tokens', 'once');
    if isempty(tok)
        fprintf('Skipping sheet "%s" (name not in x##_y## format)\n', sheet);
        continue;
    end

    x0 = str2double(tok{1});   % desired longitudinal offset in mm
    y0 = str2double(tok{2});   % desired lateral offset in mm

    data = readmatrix(filename, 'Sheet', sheet);

    if size(data,2) < 9
        fprintf('Skipping sheet "%s" (not enough columns)\n', sheet);
        continue;
    end

    % Leader A:D
    xL = data(:,2);
    yL = data(:,3);
    tL = data(:,4);

    % Follower F:I
    xF = data(:,7);
    yF = data(:,8);
    tF = data(:,9);

    % Remove NaNs
    validL = ~isnan(xL) & ~isnan(yL) & ~isnan(tL);
    validF = ~isnan(xF) & ~isnan(yF) & ~isnan(tF);

    xL = xL(validL); yL = yL(validL); tL = tL(validL);
    xF = xF(validF); yF = yF(validF); tF = tF(validF);

    if numel(xL) < 5 || numel(xF) < 5
        fprintf('Skipping sheet "%s" (too few points)\n', sheet);
        continue;
    end

    % -----------------------------------
    % Align by end of motion
    % -----------------------------------
    dxL = [0; diff(xL)];
    dxF = [0; diff(xF)];

    moveThresh = 0.5;  % mm per sample
    idxL_end = find(abs(dxL) > moveThresh, 1, 'last');
    idxF_end = find(abs(dxF) > moveThresh, 1, 'last');

    if isempty(idxL_end), idxL_end = length(tL); end
    if isempty(idxF_end), idxF_end = length(tF); end

    tL_rel = tL - tL(idxL_end);
    tF_rel = tF - tF(idxF_end);

    t_start = max(tL_rel(1), tF_rel(1));
    t_end   = min(tL_rel(end), tF_rel(end));

    if t_start >= t_end
        fprintf('Skipping sheet "%s" (no overlap after alignment)\n', sheet);
        continue;
    end

    t_common = t_start:50:t_end;

    xL_i = interp1(tL_rel, xL, t_common, 'linear');
    yL_i = interp1(tL_rel, yL, t_common, 'linear');

    xF_i = interp1(tF_rel, xF, t_common, 'linear');
    yF_i = interp1(tF_rel, yF, t_common, 'linear');

    % -----------------------------------
    % Relative-pose error
    % -----------------------------------
    err_x = -xL_i - xF_i;
    err_y = -yL_i - yF_i;

    sep_x = err_x + x0;
    sep_y = err_y + y0;

    rms_x = sqrt(mean(err_x.^2));
    rms_y = sqrt(mean(err_y.^2));
    mae_x = mean(abs(err_x));
    mae_y = mean(abs(err_y));

    all_x0(end+1) = x0;
    all_y0(end+1) = y0;
    all_rms_x(end+1) = rms_x;
    all_rms_y(end+1) = rms_y;
    all_mae_x(end+1) = mae_x;
    all_mae_y(end+1) = mae_y;

    % Save traces for surface plot
    surf_x0(end+1) = x0;
    surf_y0(end+1) = y0;
    surf_t{end+1} = t_common;
    surf_errx{end+1} = err_x;

    figure(findobj('Name','Error X vs Time'));
    plot(t_common, err_x, 'DisplayName', sheet);

    figure(findobj('Name','Error Y vs Time'));
    plot(t_common, err_y, 'DisplayName', sheet);

    % % Optional per-sheet debug plots
    % figure('Name',['Debug_' sheet]);
    % 
    % % Flip leader x sign
    % xL_i_flipped = -xL_i;
    % 
    % subplot(4,1,1);
    % plot(t_common, xL_i_flipped, 'LineWidth', 1.2); hold on;
    % plot(t_common, xF_i, 'LineWidth', 1.2);
    % grid on;
    % ylabel('x (mm)');
    % legend('Leader x (flipped)','Follower x');
    % title(['Raw x traces (leader flipped): ' sheet]);
    % 
    % subplot(4,1,2);
    % plot(t_common, yL_i, 'LineWidth', 1.2); hold on;
    % plot(t_common, yF_i, 'LineWidth', 1.2);
    % grid on;
    % ylabel('y (mm)');
    % legend('Leader y','Follower y');
    % title('Raw y traces');
    % 
    % subplot(4,1,3);
    % plot(t_common, sep_x, 'LineWidth', 1.2); hold on;
    % yline(x0, '--k');
    % grid on;
    % ylabel('sep_x (mm)');
    % legend('Actual sep_x','Desired x0');
    % 
    % subplot(4,1,4);
    % plot(t_common, sep_y, 'LineWidth', 1.2); hold on;
    % yline(y0, '--k');
    % grid on;
    % xlabel('Time relative to end (ms)');
    % ylabel('sep_y (mm)');
    % legend('Actual sep_y','Desired y0');
end

figure(findobj('Name','Error X vs Time'));
yline(0,'--k');
legend show;

figure(findobj('Name','Error Y vs Time'));
yline(0,'--k');
legend show;

% % -----------------------------------
% % Surface plots: longitudinal error over time by lateral displacement
% % One surface per commanded x0
% % -----------------------------------
% unique_x0 = unique(surf_x0);
% 
% for k = 1:length(unique_x0)
%     x0_sel = unique_x0(k);
%     idx = find(surf_x0 == x0_sel);
% 
%     if numel(idx) < 2
%         fprintf('Skipping surface for x0 = %g (need at least 2 y-levels)\n', x0_sel);
%         continue;
%     end
% 
%     y_vals = surf_y0(idx);
%     [y_vals_sorted, ord] = sort(y_vals);
%     idx = idx(ord);
% 
%     % Find common overlapping time range across all selected sheets
%     t_min_common = -inf;
%     t_max_common = inf;
% 
%     for i = 1:numel(idx)
%         t_this = surf_t{idx(i)};
%         t_min_common = max(t_min_common, min(t_this));
%         t_max_common = min(t_max_common, max(t_this));
%     end
% 
%     if t_min_common >= t_max_common
%         fprintf('Skipping surface for x0 = %g (no common time window)\n', x0_sel);
%         continue;
%     end
% 
%     t_grid = t_min_common:50:t_max_common;
% 
%     % Build Z matrix: rows = lateral offsets, cols = time
%     Z = nan(numel(idx), numel(t_grid));
% 
%     for i = 1:numel(idx)
%         Z(i,:) = interp1(surf_t{idx(i)}, surf_errx{idx(i)}, t_grid, 'linear');
%     end
% 
%     [T, Y] = meshgrid(t_grid, y_vals_sorted);
% 
%     figure('Name', sprintf('Surface_LongitudinalError_x0_%g', x0_sel));
%     surf(T, Y, Z, 'EdgeColor', 'none');
%     grid on;
%     xlabel('Time relative to end (ms)');
%     ylabel('Commanded lateral displacement y0 (mm)');
%     zlabel('Longitudinal error (mm)');
%     title(sprintf('Longitudinal error surface vs time and lateral displacement (x0 = %g mm)', x0_sel));
%     colorbar;
%     view(45,30);
% end

% -----------------------------------
% Summary plots
% -----------------------------------
figure('Name','RMS Longitudinal Error');
scatter3(all_x0, all_y0, all_rms_x, 60, all_rms_x, 'filled');
grid on;
xlabel('Commanded x offset (mm)');
ylabel('Commanded y offset (mm)');
zlabel('RMS x error (mm)');
title('RMS longitudinal error across offsets');
colorbar;

figure('Name','RMS Lateral Error');
scatter3(all_x0, all_y0, all_rms_y, 60, all_rms_y, 'filled');
grid on;
xlabel('Commanded x offset (mm)');
ylabel('Commanded y offset (mm)');
zlabel('RMS y error (mm)');
title('RMS lateral error across offsets');
colorbar;

figure('Name','MAE Longitudinal Error');
scatter3(all_x0, all_y0, all_mae_x, 60, all_mae_x, 'filled');
grid on;
xlabel('Commanded x offset (mm)');
ylabel('Commanded y offset (mm)');
zlabel('MAE x error (mm)');
title('Mean absolute longitudinal error');
colorbar;

figure('Name','MAE Lateral Error');
scatter3(all_x0, all_y0, all_mae_y, 60, all_mae_y, 'filled');
grid on;
xlabel('Commanded x offset (mm)');
ylabel('Commanded y offset (mm)');
zlabel('MAE y error (mm)');
title('Mean absolute lateral error');
colorbar;

% % Use exactly this style:
% % 
% % x50_y0
% % x100_y0
% % x100_y20
% % x100_y-20
% 
% clear; clc; close all;
% 
% filename = 'XY_offset2.xlsx';   % <-- change this
% [~, sheetNames] = xlsfinfo(filename);
% 
% all_x0 = [];
% all_y0 = [];
% all_rms_x = [];
% all_rms_y = [];
% all_mae_x = [];
% all_mae_y = [];
% 
% figure('Name','Error X vs Time'); hold on; grid on;
% xlabel('Time relative to end (ms)');
% ylabel('Longitudinal error (mm)');
% title('Longitudinal error vs time');
% 
% figure('Name','Error Y vs Time'); hold on; grid on;
% xlabel('Time relative to end (ms)');
% ylabel('Lateral error (mm)');
% title('Lateral error vs time');
% 
% for s = 1:length(sheetNames)
%     sheet = sheetNames{s};
% 
%     % Parse sheet name like x50_y-20
%     tok = regexp(sheet, 'x([-+]?\d+)_y([-+]?\d+)', 'tokens', 'once');
%     if isempty(tok)
%         fprintf('Skipping sheet "%s" (name not in x##_y## format)\n', sheet);
%         continue;
%     end
% 
%     x0 = str2double(tok{1});   % desired longitudinal offset in mm
%     y0 = str2double(tok{2});   % desired lateral offset in mm
% 
%     data = readmatrix(filename, 'Sheet', sheet);
% 
%     if size(data,2) < 9
%         fprintf('Skipping sheet "%s" (not enough columns)\n', sheet);
%         continue;
%     end
% 
%     % Leader A:D
%     xL = data(:,2);
%     yL = data(:,3);
%     tL = data(:,4);
% 
%     % Follower F:I
%     xF = data(:,7);
%     yF = data(:,8);
%     tF = data(:,9);
% 
%     % Remove NaNs
%     validL = ~isnan(xL) & ~isnan(yL) & ~isnan(tL);
%     validF = ~isnan(xF) & ~isnan(yF) & ~isnan(tF);
% 
%     xL = xL(validL); yL = yL(validL); tL = tL(validL);
%     xF = xF(validF); yF = yF(validF); tF = tF(validF);
% 
%     if numel(xL) < 5 || numel(xF) < 5
%         fprintf('Skipping sheet "%s" (too few points)\n', sheet);
%         continue;
%     end
% 
%     % -----------------------------------
%     % Align by end of motion
%     % -----------------------------------
%     dxL = [0; diff(xL)];
%     dxF = [0; diff(xF)];
% 
%     moveThresh = 0.5;  % mm per sample
%     idxL_end = find(abs(dxL) > moveThresh, 1, 'last');
%     idxF_end = find(abs(dxF) > moveThresh, 1, 'last');
% 
%     if isempty(idxL_end), idxL_end = length(tL); end
%     if isempty(idxF_end), idxF_end = length(tF); end
% 
%     tL_rel = tL - tL(idxL_end);
%     tF_rel = tF - tF(idxF_end);
% 
%     t_start = max(tL_rel(1), tF_rel(1));
%     t_end   = min(tL_rel(end), tF_rel(end));
% 
%     if t_start >= t_end
%         fprintf('Skipping sheet "%s" (no overlap after alignment)\n', sheet);
%         continue;
%     end
% 
%     t_common = t_start:50:t_end;
% 
%     xL_i = interp1(tL_rel, xL, t_common, 'linear');
%     yL_i = interp1(tL_rel, yL, t_common, 'linear');
% 
%     xF_i = interp1(tF_rel, xF, t_common, 'linear');
%     yF_i = interp1(tF_rel, yF, t_common, 'linear');
% 
%     % -----------------------------------
%     % Relative-pose error
%     %
%     % Using follower frame convention from your setup:
%     % longitudinal separation = x0 - xL - xF
%     % lateral relative error  = y0 - yL - yF
%     %
%     % So errors relative to commanded offsets are:
%     % err_x = actual_longitudinal_sep - x0 = -xL - xF
%     % err_y = actual_lateral_sep      - y0 = -yL - yF
%     %
%     % If you want to compare against commanded y0 directly as a world quantity,
%     % use sep_y = y0 - yL - yF and err_y = sep_y - y0 = -yL - yF.
%     %
%     % So commanded offsets drop out once both logs are zeroed at their own origins.
%     % -----------------------------------
%     err_x = -xL_i - xF_i;
%     err_y = -yL_i - yF_i;
% 
%     sep_x = err_x + x0;
%     sep_y = err_y + y0;
% 
%     rms_x = sqrt(mean(err_x.^2));
%     rms_y = sqrt(mean(err_y.^2));
%     mae_x = mean(abs(err_x));
%     mae_y = mean(abs(err_y));
% 
%     all_x0(end+1) = x0;
%     all_y0(end+1) = y0;
%     all_rms_x(end+1) = rms_x;
%     all_rms_y(end+1) = rms_y;
%     all_mae_x(end+1) = mae_x;
%     all_mae_y(end+1) = mae_y;
% 
%     figure(findobj('Name','Error X vs Time'));
%     plot(t_common, err_x, 'DisplayName', sheet);
% 
%     figure(findobj('Name','Error Y vs Time'));
%     plot(t_common, err_y, 'DisplayName', sheet);
% 
%     % Optional per-sheet debug plots
%     figure('Name',['Debug_' sheet]);
% 
%     subplot(3,1,1);
%     plot(t_common, xL_i, 'LineWidth', 1.2); hold on;
%     plot(t_common, xF_i, 'LineWidth', 1.2);
%     grid on;
%     ylabel('x (mm)');
%     legend('Leader x','Follower x');
%     title(['Raw x traces: ' sheet]);
% 
%     subplot(3,1,2);
%     plot(t_common, sep_x, 'LineWidth', 1.2); hold on;
%     yline(x0, '--k');
%     grid on;
%     ylabel('sep_x (mm)');
%     legend('Actual sep_x','Desired x0');
% 
%     subplot(3,1,3);
%     plot(t_common, sep_y, 'LineWidth', 1.2); hold on;
%     yline(y0, '--k');
%     grid on;
%     xlabel('Time relative to end (ms)');
%     ylabel('sep_y (mm)');
%     legend('Actual sep_y','Desired y0');
% end
% 
% figure(findobj('Name','Error X vs Time'));
% yline(0,'--k');
% legend show;
% 
% figure(findobj('Name','Error Y vs Time'));
% yline(0,'--k');
% legend show;
% 
% % -----------------------------------
% % Summary plots
% % -----------------------------------
% figure('Name','RMS Longitudinal Error');
% scatter3(all_x0, all_y0, all_rms_x, 60, all_rms_x, 'filled');
% % surf(all_x0, all_y0, all_rms_x);
% grid on;
% xlabel('Commanded x offset (mm)');
% ylabel('Commanded y offset (mm)');
% zlabel('RMS x error (mm)');
% title('RMS longitudinal error across offsets');
% colorbar;
% 
% figure('Name','RMS Lateral Error');
% scatter3(all_x0, all_y0, all_rms_y, 60, all_rms_y, 'filled');
% grid on;
% xlabel('Commanded x offset (mm)');
% ylabel('Commanded y offset (mm)');
% zlabel('RMS y error (mm)');
% title('RMS lateral error across offsets');
% colorbar;
% 
% figure('Name','MAE Longitudinal Error');
% scatter3(all_x0, all_y0, all_mae_x, 60, all_mae_x, 'filled');
% grid on;
% xlabel('Commanded x offset (mm)');
% ylabel('Commanded y offset (mm)');
% zlabel('MAE x error (mm)');
% title('Mean absolute longitudinal error');
% colorbar;
% 
% figure('Name','MAE Lateral Error');
% scatter3(all_x0, all_y0, all_mae_y, 60, all_mae_y, 'filled');
% grid on;
% xlabel('Commanded x offset (mm)');
% ylabel('Commanded y offset (mm)');
% zlabel('MAE y error (mm)');
% title('Mean absolute lateral error');
% colorbar;