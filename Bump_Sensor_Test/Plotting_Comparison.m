clear; clc; close all;

filename = 'XY_offset2.xlsx';   % <-- change this

[~, sheetNames] = xlsfinfo(filename);

all_d0  = [];
all_rms_x = [];
all_rms_y = [];

figure('Name','Error X vs Time'); hold on; grid on;
xlabel('Time relative to end (ms)');
ylabel('Error X (mm)');
title('Longitudinal spacing error');

figure('Name','Error Y vs Time'); hold on; grid on;
xlabel('Time relative to end (ms)');
ylabel('Error Y (mm)');
title('Lateral error');

for s = 1:length(sheetNames)

    sheet = sheetNames{s};
    d0 = str2double(sheet);

    if isnan(d0)
        continue;
    end

    data = readmatrix(filename, 'Sheet', sheet);

    if size(data,2) < 9
        continue;
    end

    % Leader
    xL = data(:,2); yL = data(:,3); tL = data(:,4);

    % Follower
    xF = data(:,7); yF = data(:,8); tF = data(:,9);

    % Clean NaNs
    validL = ~isnan(xL) & ~isnan(tL);
    validF = ~isnan(xF) & ~isnan(tF);

    xL = xL(validL); yL = yL(validL); tL = tL(validL);
    xF = xF(validF); yF = yF(validF); tF = tF(validF);

    % -------------------------
    % Align by end of motion
    % -------------------------
    dxL = [0; diff(xL)];
    dxF = [0; diff(xF)];

    thresh = 0.5;

    idxL_end = find(abs(dxL) > thresh, 1, 'last');
    idxF_end = find(abs(dxF) > thresh, 1, 'last');

    if isempty(idxL_end), idxL_end = length(tL); end
    if isempty(idxF_end), idxF_end = length(tF); end

    tL_rel = tL - tL(idxL_end);
    tF_rel = tF - tF(idxF_end);

    % -------------------------
    % Interpolate
    % -------------------------
    t_start = max(tL_rel(1), tF_rel(1));
    t_end   = min(tL_rel(end), tF_rel(end));

    t_common = t_start:50:t_end;

    xL_i = interp1(tL_rel, xL, t_common);
    yL_i = interp1(tL_rel, yL, t_common);

    xF_i = interp1(tF_rel, xF, t_common);
    yF_i = interp1(tF_rel, yF, t_common);

    % -------------------------
    % Errors
    % -------------------------
    error_x = -xL_i - xF_i;
    error_y = -yL_i - yF_i;

    % RMS metrics
    rms_x = sqrt(mean(error_x.^2));
    rms_y = sqrt(mean(error_y.^2));

    all_d0(end+1)   = d0;
    all_rms_x(end+1) = rms_x;
    all_rms_y(end+1) = rms_y;

    % -------------------------
    % Plot
    % -------------------------
    figure(findobj('Name','Error X vs Time'));
    plot(t_common, error_x, 'DisplayName', sprintf('%g mm', d0));

    figure(findobj('Name','Error Y vs Time'));
    plot(t_common, error_y, 'DisplayName', sprintf('%g mm', d0));

end

% Finalise plots
figure(findobj('Name','Error X vs Time'));
yline(0,'--k');
legend show;

figure(findobj('Name','Error Y vs Time'));
yline(0,'--k');
legend show;

% -------------------------
% Sort results
% -------------------------
[all_d0, idx] = sort(all_d0);
all_rms_x = all_rms_x(idx);
all_rms_y = all_rms_y(idx);

% -------------------------
% Summary plots
% -------------------------
figure;
plot(all_d0, all_rms_x, '-o','LineWidth',1.5); hold on;
plot(all_d0, all_rms_y, '-o','LineWidth',1.5);
grid on;
xlabel('Initial separation (mm)');
ylabel('RMS error (mm)');
legend('RMS X error','RMS Y error');
title('Tracking error vs initial separation');

% Multisheet only x error plotting
% clear; clc; close all;
% 
% filename = 'leader_follower_tests.xlsx';   % <-- change this
% 
% [~, sheetNames] = xlsfinfo(filename);
% 
% all_d0  = [];
% all_rms = [];
% all_mae = [];
% all_fin = [];
% 
% figure('Name','Separation vs Time'); hold on; grid on;
% xlabel('Time relative to end (ms)');
% ylabel('Separation (mm)');
% title('Actual separation vs time');
% 
% figure('Name','Spacing Error vs Time'); hold on; grid on;
% xlabel('Time relative to end (ms)');
% ylabel('Spacing error (mm)');
% title('Spacing error vs time');
% 
% for s = 1:length(sheetNames)
% 
%     sheet = sheetNames{s};
%     d0 = str2double(sheet);   % sheet name is separation in mm
% 
%     if isnan(d0)
%         fprintf('Skipping sheet "%s" (not numeric)\n', sheet);
%         continue;
%     end
% 
%     data = readmatrix(filename, 'Sheet', sheet);
% 
%     % Need at least columns A:D and F:I
%     if size(data,2) < 9
%         fprintf('Skipping sheet "%s" (not enough columns)\n', sheet);
%         continue;
%     end
% 
%     % -------------------------
%     % Leader in A:D
%     % -------------------------
%     xL = data(:,2);
%     yL = data(:,3);
%     tL = data(:,4);
% 
%     % -------------------------
%     % Follower in F:I
%     % -------------------------
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
%     % -------------------------
%     % Align by end of motion, not start
%     % Find last meaningful movement sample
%     % -------------------------
%     dxL = [0; diff(xL)];
%     dxF = [0; diff(xF)];
% 
%     moveThresh = 0.5;  % mm/sample
%     idxL_end = find(abs(dxL) > moveThresh, 1, 'last');
%     idxF_end = find(abs(dxF) > moveThresh, 1, 'last');
% 
%     if isempty(idxL_end), idxL_end = length(tL); end
%     if isempty(idxF_end), idxF_end = length(tF); end
% 
%     tL_rel = tL - tL(idxL_end);
%     tF_rel = tF - tF(idxF_end);
% 
%     % -------------------------
%     % Common overlapping time axis
%     % Time will be negative up to 0
%     % -------------------------
%     t_start = max(tL_rel(1), tF_rel(1));
%     t_end   = min(tL_rel(end), tF_rel(end));
% 
%     if t_start >= t_end
%         fprintf('Skipping sheet "%s" (no overlapping time)\n', sheet);
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
%     % -------------------------
%     % Separation / error
%     % Leader moving backward => xL negative increases gap
%     % Follower moving forward => xF positive decreases gap
%     % -------------------------
%     sep_x = d0 - xL_i - xF_i;
%     err_x = sep_x - d0;
% 
%     % Optional lateral mismatch
%     err_y = -yL_i - yF_i; %#ok<NASGU>
% 
%     % Summary metrics
%     rms_err   = sqrt(mean(err_x.^2));
%     mae_err   = mean(abs(err_x));
%     final_err = err_x(end);
% 
%     all_d0(end+1)  = d0;
%     all_rms(end+1) = rms_err;
%     all_mae(end+1) = mae_err;
%     all_fin(end+1) = final_err;
% 
%     % -------------------------
%     % Plot this sheet
%     % -------------------------
%     figure(findobj('Name','Separation vs Time'));
%     plot(t_common, sep_x, 'DisplayName', sprintf('%g mm', d0));
% 
%     figure(findobj('Name','Spacing Error vs Time'));
%     plot(t_common, err_x, 'DisplayName', sprintf('%g mm', d0));
% 
%     fprintf('Sheet %s: RMS=%.2f mm, MAE=%.2f mm, Final=%.2f mm\n', ...
%         sheet, rms_err, mae_err, final_err);
% end
% 
% % Finish legends
% figure(findobj('Name','Separation vs Time'));
% legend('show', 'Location', 'best');
% 
% figure(findobj('Name','Spacing Error vs Time'));
% yline(0, '--k');
% legend('show', 'Location', 'best');
% 
% % -------------------------
% % Sort summary results by separation
% % -------------------------
% [all_d0, idx] = sort(all_d0);
% all_rms = all_rms(idx);
% all_mae = all_mae(idx);
% all_fin = all_fin(idx);
% 
% % -------------------------
% % Summary plots
% % -------------------------
% figure('Name','RMS Error vs Separation');
% plot(all_d0, all_rms, '-o', 'LineWidth', 1.5);
% grid on;
% xlabel('Initial separation (mm)');
% ylabel('RMS spacing error (mm)');
% title('RMS spacing error vs initial separation');
% 
% figure('Name','MAE Error vs Separation');
% plot(all_d0, all_mae, '-o', 'LineWidth', 1.5);
% grid on;
% xlabel('Initial separation (mm)');
% ylabel('Mean absolute spacing error (mm)');
% title('Mean absolute spacing error vs initial separation');
% 
% figure('Name','Final Error vs Separation');
% plot(all_d0, all_fin, '-o', 'LineWidth', 1.5);
% grid on;
% xlabel('Initial separation (mm)');
% ylabel('Final spacing error (mm)');
% title('Final spacing error vs initial separation');

%% Single Plot
% clear; clc;
% 
% filename = 'leader_follower_tests.xlsx';
% sheet = '100';   % example
% d0 = str2double(sheet);   % mm
% 
% data = readmatrix(filename, 'Sheet', sheet);
% 
% % Leader A:D
% xL = data(:,2);
% yL = data(:,3);
% tL = data(:,4);
% 
% % Follower F:I
% xF = data(:,7);
% yF = data(:,8);
% tF = data(:,9);
% 
% % Remove NaNs
% validL = ~isnan(xL) & ~isnan(tL);
% validF = ~isnan(xF) & ~isnan(tF);
% 
% xL = xL(validL); yL = yL(validL); tL = tL(validL);
% xF = xF(validF); yF = yF(validF); tF = tF(validF);
% 
% % -------------------------
% % Align by end time
% % -------------------------
% % Option A: raw end alignment
% tL_rel = tL - tL(end);
% tF_rel = tF - tF(end);
% 
% % If follower has a 200 ms stop timeout, you may prefer:
% % LOST_TIMEOUT_MS = 200;
% % tF_rel = tF - (tF(end) - LOST_TIMEOUT_MS);
% 
% % -------------------------
% % Common overlapping time axis
% % -------------------------
% t_start = max(tL_rel(1), tF_rel(1));
% t_end   = min(tL_rel(end), tF_rel(end));   % usually near 0
% t_common = t_start:50:t_end;
% 
% xL_i = interp1(tL_rel, xL, t_common, 'linear');
% xF_i = interp1(tF_rel, xF, t_common, 'linear');
% yL_i = interp1(tL_rel, yL, t_common, 'linear');
% yF_i = interp1(tF_rel, yF, t_common, 'linear');
% 
% % -------------------------
% % Separation and error
% % -------------------------
% sep_x = d0 - xL_i - xF_i;
% err_x = sep_x - d0;
% 
% % If you want lateral mismatch too:
% err_y = -yL_i - yF_i;
% 
% % -------------------------
% % Plots
% % -------------------------
% figure;
% subplot(3,1,1);
% plot(t_common, xL_i, 'LineWidth', 1.5); hold on;
% plot(t_common, xF_i, 'LineWidth', 1.5);
% grid on;
% legend('Leader x','Follower x');
% ylabel('mm');
% title(sprintf('Raw x traces, sheet %s', sheet));
% 
% subplot(3,1,2);
% plot(t_common, sep_x, 'LineWidth', 1.5); hold on;
% yline(d0, '--');
% grid on;
% ylabel('mm');
% legend('Actual separation','Desired separation');
% 
% subplot(3,1,3);
% plot(t_common, err_x, 'LineWidth', 1.5); hold on;
% yline(0, '--');
% grid on;
% xlabel('Time relative to end event (ms)');
% ylabel('mm');
% legend('Spacing error','Zero error');


% clear; clc;
% 
% filename = 'leader_follower_tests.xlsx';   % <-- change this
% 
% [~, sheetNames] = xlsfinfo(filename);
% 
% % Store results
% all_d0 = [];
% all_rms = [];
% 
% figure; hold on; grid on;
% title('Spacing error vs time');
% xlabel('Time (ms)');
% ylabel('Error (mm)');
% 
% for s = 1:length(sheetNames)
% 
%     sheet = sheetNames{s};
%     d0 = str2double(sheet);   % sheet name = initial separation (mm)
% 
%     if isnan(d0)
%         continue; % skip non-numeric sheets
%     end
% 
%     % -------------------------
%     % Read data
%     % -------------------------
%     data = readmatrix(filename, 'Sheet', sheet);
% 
%     % Leader (A:D)
%     xL = data(:,2);
%     yL = data(:,3);
%     tL = data(:,4);
% 
%     % Follower (F:I)
%     xF = data(:,7);
%     yF = data(:,8);
%     tF = data(:,9);
% 
%     % Remove NaNs (in case columns uneven)
%     validL = ~isnan(xL);
%     validF = ~isnan(xF);
% 
%     xL = xL(validL); yL = yL(validL); tL = tL(validL);
%     xF = xF(validF); yF = yF(validF); tF = tF(validF);
% 
%     % -------------------------
%     % Align time (start at motion)
%     % -------------------------
%     thresh = 1; % mm
% 
%     idxL = find(abs(xL - xL(1)) > thresh, 1);
%     idxF = find(abs(xF - xF(1)) > thresh, 1);
% 
%     if isempty(idxL), idxL = 1; end
%     if isempty(idxF), idxF = 1; end
% 
%     xL = xL(idxL:end) - xL(idxL);
%     yL = yL(idxL:end) - yL(idxL);
%     tL = tL(idxL:end) - tL(idxL);
% 
%     xF = xF(idxF:end) - xF(idxF);
%     yF = yF(idxF:end) - yF(idxF);
%     tF = tF(idxF:end) - tF(idxF);
% 
%     % -------------------------
%     % Convert to common frame
%     % (Follower frame is world)
%     % -------------------------
%     xLw = d0 - xL;
%     yLw = -yL;
% 
%     xFw = xF;
%     yFw = yF;
% 
%     % -------------------------
%     % Interpolate to common time
%     % -------------------------
%     t_end = min(tL(end), tF(end));
%     t_common = 0:50:t_end;
% 
%     xL_i = interp1(tL, xLw, t_common);
%     yL_i = interp1(tL, yLw, t_common);
% 
%     xF_i = interp1(tF, xFw, t_common);
%     yF_i = interp1(tF, yFw, t_common);
% 
%     % -------------------------
%     % Compute errors
%     % -------------------------
%     sep_x = xL_i - xF_i;
%     error_x = sep_x - d0;
% 
%     % summary metric (RMS)
%     rms_err = sqrt(mean(error_x.^2));
% 
%     % store
%     all_d0(end+1) = d0;
%     all_rms(end+1) = rms_err;
% 
%     % plot error vs time
%     plot(t_common, error_x, 'DisplayName', sprintf('%d mm', d0));
% 
% end
% 
% legend;
% 
% % -------------------------
% % Plot error vs separation
% % -------------------------
% figure;
% plot(all_d0, all_rms, '-o');
% grid on;
% xlabel('Initial separation (mm)');
% ylabel('RMS spacing error (mm)');
% title('Error vs initial separation');
% 
% %%
% figure()
% plot(t_common, xL_i, t_common, xF_i);
% legend('Leader x','Follower x');
% grid on;
% 
% figure()
% plot(t_common, -xL_i, t_common, xF_i);
% legend('-Leader x','Follower x');
% grid on;
% leader_term   = xL_i;
% follower_term =  xF_i;
% err_x = leader_term - follower_term;
% 
% figure;
% plot(t_common, leader_term, 'LineWidth', 1.5); hold on;
% plot(t_common, follower_term, 'LineWidth', 1.5);
% plot(t_common, err_x, 'LineWidth', 1.5);
% grid on;
% legend('-x_L','x_F','error_x');
% xlabel('Time (ms)');
% ylabel('mm');
% title('Spacing error components');
% % %% Leader-Follower Analysis
% % % End-aligned error analysis
% % % Full path plot in a common physical frame
% % % Leader starts at x = d, follower starts at x = 0
% % % Leader x is flipped internally because reverse motion was logged as negative x
% % 
% % clear; clc; close all;
% % 
% % %% ---------------- USER SETTINGS ----------------
% % filename = 'leader_follower_tests.xlsx';   % <-- change to your workbook
% % dt_ms = 50;                                % logging interval
% % lostTimeout_ms = 200;                      % follower LOST_TIMEOUT_MS
% % leaderPrestart_ms = 800;                   % leader PRESTART_BEACON_MS
% % moveThresh_mm = 2.0;                       % threshold for declaring follower motion
% % 
% % savePlots = false;
% % plotFolder = 'matlab_plots';
% % 
% % if savePlots && ~exist(plotFolder, 'dir')
% %     mkdir(plotFolder);
% % end
% % 
% % %% ---------------- LOAD SHEETS ----------------
% % [~, sheetNames] = xlsfinfo(filename);
% % if isempty(sheetNames)
% %     error('No sheets found in workbook.');
% % end
% % 
% % summary = table();
% % 
% % for s = 1:numel(sheetNames)
% %     sheetName = sheetNames{s};
% %     d_mm = str2double(sheetName);
% % 
% %     if isnan(d_mm)
% %         warning('Skipping sheet "%s" because sheet name is not numeric.', sheetName);
% %         continue;
% %     end
% % 
% %     raw = readcell(filename, 'Sheet', sheetName);
% % 
% %     % Remove fully empty rows
% %     keepRows = ~all(cellfun(@(x) isempty(x) || (isstring(x) && strlength(x)==0), raw), 2);
% %     raw = raw(keepRows,:);
% % 
% %     % Expected layout:
% %     % A:E   leader  -> i, time_ms, x_mm, y_mm, theta_rad
% %     % G:K   follower-> i, time_ms, x_mm, y_mm, theta_rad
% %     if size(raw,2) < 11
% %         warning('Skipping sheet "%s": expected at least 11 columns.', sheetName);
% %         continue;
% %     end
% % 
% %     leader = raw(:,1:5);
% %     follower = raw(:,7:11);
% % 
% %     leaderKeep = cellfun(@isnumeric, leader(:,2));
% %     followerKeep = cellfun(@isnumeric, follower(:,2));
% % 
% %     leader = leader(leaderKeep,:);
% %     follower = follower(followerKeep,:);
% % 
% %     if isempty(leader) || isempty(follower)
% %         warning('Skipping sheet "%s": no usable leader/follower data.', sheetName);
% %         continue;
% %     end
% % 
% %     L = cell2mat(leader);
% %     F = cell2mat(follower);
% % 
% %     %% ---------------- EXTRACT RAW DATA ----------------
% %     % Leader
% %     L_time = L(:,2);
% %     L_x_raw = L(:,3);
% %     L_y_raw = L(:,4);
% %     L_th = L(:,5);
% % 
% %     % Follower
% %     F_time = F(:,2);
% %     F_x_raw = F(:,3);
% %     F_y_raw = F(:,4);
% %     F_th = F(:,5);
% % 
% %     % Flip leader x so reverse travel becomes positive
% %     L_x = -L_x_raw;
% %     L_y = L_y_raw;
% % 
% %     F_x = F_x_raw;
% %     F_y = F_y_raw;
% % 
% %     %% ---------------- COMMON PHYSICAL FRAME FOR PATH PLOT ----------------
% %     % Put follower start at (0,0)
% %     % Put leader start at (d,0)
% %     L_x_plot = d_mm + (L_x - L_x(1));
% %     L_y_plot = L_y - L_y(1);
% % 
% %     F_x_plot = F_x - F_x(1);
% %     F_y_plot = F_y - F_y(1);
% % 
% %     %% ---------------- START DELAY / REACTION DELAY ----------------
% %     % Leader start time is defined as the first sample = 0 ms
% %     leader_start_time_ms = 0;
% % 
% %     % Follower movement start = first sample with enough displacement
% %     F_start_idx = find(abs(F_x - F_x(1)) > moveThresh_mm | ...
% %                        abs(F_y - F_y(1)) > moveThresh_mm, 1, 'first');
% %     if isempty(F_start_idx)
% %         F_start_idx = 1;
% %     end
% % 
% %     follower_start_time_ms = F_time(F_start_idx) - L_time(1);
% %     start_delay_ms = follower_start_time_ms;
% %     start_delay_samples = round(start_delay_ms / dt_ms);
% % 
% %     % Reaction delay = follower start minus leader actual motion start
% %     reaction_delay_ms = follower_start_time_ms - leaderPrestart_ms;
% %     reaction_delay_samples = round(reaction_delay_ms / dt_ms);
% % 
% %     %% ---------------- END ALIGNMENT FOR ERROR ANALYSIS ----------------
% %     % Leader stop event = final leader timestamp
% %     % Follower stop-reference = final follower timestamp - LOST_TIMEOUT_MS
% %     L_stop_ref = L_time(end);
% %     F_stop_ref = F_time(end) - lostTimeout_ms;
% % 
% %     L_tstop = L_time - L_stop_ref;   % ends at 0
% %     F_tstop = F_time - F_stop_ref;   % corrected to estimated leader stop event
% % 
% %     % Build end-aligned full comparison grid over leader duration
% %     tStart = min(L_tstop);           % most negative leader time
% %     tCommon = (tStart:dt_ms:0)';
% % 
% %     % Interpolate full paths in common physical frame
% %     L_xi = interp1(L_tstop, L_x_plot, tCommon, 'linear', 'extrap');
% %     L_yi = interp1(L_tstop, L_y_plot, tCommon, 'linear', 'extrap');
% % 
% %     F_xi = interp1(F_tstop, F_x_plot, tCommon, 'linear', NaN);
% %     F_yi = interp1(F_tstop, F_y_plot, tCommon, 'linear', NaN);
% % 
% %     % Fill missing follower samples with (0,0)
% %     % This creates the "stationary before movement" section explicitly
% %     F_xi(isnan(F_xi)) = 0;
% %     F_yi(isnan(F_yi)) = 0;
% % 
% %     %% ---------------- DESIRED FOLLOWER PATH ----------------
% %     % Leader path already includes the initial separation d in the common frame.
% %     % Since follower starts at 0 and leader starts at d, the desired follower
% %     % path is simply the leader path shifted back by d.
% %     x_des = L_xi - d_mm;
% %     y_des = L_yi;
% % 
% %     %% ---------------- ERRORS OVER TIME ----------------
% %     % Longitudinal tracking error relative to desired gap path
% %     ex = F_xi - x_des;
% % 
% %     % Lateral tracking error
% %     ey = F_yi - y_des;
% % 
% %     % Total path error
% %     etotal = sqrt(ex.^2 + ey.^2);
% % 
% %     % Gap error
% %     gapErr = (L_xi - F_xi) - d_mm;
% % 
% %     %% ---------------- SUMMARY METRICS ----------------
% %     RMS_ex = sqrt(mean(ex.^2));
% %     RMS_ey = sqrt(mean(ey.^2));
% %     RMS_etotal = sqrt(mean(etotal.^2));
% % 
% %     MAX_ex = max(abs(ex));
% %     MAX_ey = max(abs(ey));
% %     MAX_etotal = max(etotal);
% % 
% %     RMS_gap = sqrt(mean(gapErr.^2));
% %     MAX_gap = max(abs(gapErr));
% % 
% %     leader_rms_lat = sqrt(mean(L_yi.^2));
% %     leader_max_lat = max(abs(L_yi));
% % 
% %     follower_rms_lat = sqrt(mean(F_yi.^2));
% %     follower_max_lat = max(abs(F_yi));
% % 
% %     %% ---------------- PLOTS ----------------
% %     figure('Name', sprintf('d = %g mm', d_mm), 'Color', 'w', ...
% %         'Position', [80 80 1450 900]);
% %     tiledlayout(3,2);
% % 
% %     % 1. Full path plot in common physical frame
% %     nexttile;
% %     plot(L_x_plot, L_y_plot, 'LineWidth', 1.8); hold on;
% %     plot(F_x_plot, F_y_plot, 'LineWidth', 1.8);
% %     xlabel('x (mm)');
% %     ylabel('y (mm)');
% %     title(sprintf('Full Paths in Common Frame, d = %g mm', d_mm));
% %     legend('Leader', 'Follower', 'Location', 'best');
% %     axis equal;
% %     grid on;
% % 
% %     % 2. Total error over time
% %     nexttile;
% %     plot(tCommon, etotal, 'k', 'LineWidth', 1.8);
% %     xlabel('Time relative to leader stop (ms)');
% %     ylabel('Total path error (mm)');
% %     title('Total Path Error vs Time');
% %     grid on;
% % 
% %     % 3. Longitudinal error
% %     nexttile;
% %     plot(tCommon, ex, 'LineWidth', 1.8);
% %     xlabel('Time relative to leader stop (ms)');
% %     ylabel('Longitudinal error e_x (mm)');
% %     title('Longitudinal Error vs Time');
% %     grid on;
% % 
% %     % 4. Lateral error
% %     nexttile;
% %     plot(tCommon, ey, 'LineWidth', 1.8);
% %     xlabel('Time relative to leader stop (ms)');
% %     ylabel('Lateral error e_y (mm)');
% %     title('Lateral Error vs Time');
% %     grid on;
% % 
% %     % 5. Gap error
% %     nexttile;
% %     plot(tCommon, gapErr, 'LineWidth', 1.8);
% %     xlabel('Time relative to leader stop (ms)');
% %     ylabel('Gap error (mm)');
% %     title(sprintf('Gap Error (desired d = %g mm)', d_mm));
% %     grid on;
% % 
% %     % 6. Text summary
% %     nexttile;
% %     axis off;
% %     text(0.02, 0.88, sprintf('Leader start time = %.0f ms', leader_start_time_ms), 'FontSize', 11);
% %     text(0.02, 0.76, sprintf('Follower start time = %.0f ms', follower_start_time_ms), 'FontSize', 11);
% %     text(0.02, 0.64, sprintf('Follower delay = %.0f ms (%d samples)', ...
% %         start_delay_ms, start_delay_samples), 'FontSize', 11);
% %     text(0.02, 0.52, sprintf('Reaction delay = %.0f ms (%d samples)', ...
% %         reaction_delay_ms, reaction_delay_samples), 'FontSize', 11);
% %     text(0.02, 0.38, sprintf('RMS e_x = %.2f mm', RMS_ex), 'FontSize', 11);
% %     text(0.02, 0.28, sprintf('RMS e_y = %.2f mm', RMS_ey), 'FontSize', 11);
% %     text(0.02, 0.18, sprintf('RMS e_total = %.2f mm', RMS_etotal), 'FontSize', 11);
% %     text(0.02, 0.08, sprintf('MAX e_total = %.2f mm', MAX_etotal), 'FontSize', 11);
% % 
% %     sgtitle(sprintf('Leader-Follower Analysis, d = %g mm', d_mm));
% % 
% %     if savePlots
% %         saveas(gcf, fullfile(plotFolder, sprintf('analysis_d_%g.png', d_mm)));
% %     end
% % 
% %     %% ---------------- STORE SUMMARY ----------------
% %     summary = [summary; table( ...
% %         d_mm, ...
% %         leader_start_time_ms, ...
% %         follower_start_time_ms, ...
% %         start_delay_ms, ...
% %         start_delay_samples, ...
% %         reaction_delay_ms, ...
% %         reaction_delay_samples, ...
% %         RMS_ex, RMS_ey, RMS_etotal, ...
% %         MAX_ex, MAX_ey, MAX_etotal, ...
% %         RMS_gap, MAX_gap, ...
% %         leader_rms_lat, leader_max_lat, ...
% %         follower_rms_lat, follower_max_lat, ...
% %         'VariableNames', { ...
% %         'd_mm', ...
% %         'LeaderStart_ms', ...
% %         'FollowerStart_ms', ...
% %         'FollowerDelay_ms', ...
% %         'FollowerDelay_samples', ...
% %         'ReactionDelay_ms', ...
% %         'ReactionDelay_samples', ...
% %         'RMS_ex_mm', 'RMS_ey_mm', 'RMS_etotal_mm', ...
% %         'MAX_ex_mm', 'MAX_ey_mm', 'MAX_etotal_mm', ...
% %         'RMS_gap_mm', 'MAX_gap_mm', ...
% %         'Leader_RMS_Lateral_mm', 'Leader_MAX_Lateral_mm', ...
% %         'Follower_RMS_Lateral_mm', 'Follower_MAX_Lateral_mm'})];
% % end
% % 
% % %% ---------------- SUMMARY OUTPUT ----------------
% % disp(summary);
% % 
% % %% ---------------- SUMMARY FIGURE ----------------
% % if ~isempty(summary)
% %     summary = sortrows(summary, 'd_mm');
% % 
% %     figure('Name', 'Summary Across Distances', 'Color', 'w', ...
% %         'Position', [100 100 1450 850]);
% %     tiledlayout(2,3);
% % 
% %     nexttile;
% %     plot(summary.d_mm, summary.FollowerDelay_ms, '-o', 'LineWidth', 1.8); hold on;
% %     plot(summary.d_mm, summary.ReactionDelay_ms, '-o', 'LineWidth', 1.8);
% %     xlabel('Separation d (mm)');
% %     ylabel('Delay (ms)');
% %     title('Start Delay vs Separation');
% %     legend('Follower delay', 'Reaction delay', 'Location', 'best');
% %     grid on;
% % 
% %     nexttile;
% %     plot(summary.d_mm, summary.RMS_etotal_mm, '-o', 'LineWidth', 1.8);
% %     xlabel('Separation d (mm)');
% %     ylabel('RMS total path error (mm)');
% %     title('RMS Total Path Error vs Separation');
% %     grid on;
% % 
% %     nexttile;
% %     plot(summary.d_mm, summary.MAX_etotal_mm, '-o', 'LineWidth', 1.8);
% %     xlabel('Separation d (mm)');
% %     ylabel('Max total path error (mm)');
% %     title('Max Total Path Error vs Separation');
% %     grid on;
% % 
% %     nexttile;
% %     plot(summary.d_mm, summary.RMS_ex_mm, '-o', 'LineWidth', 1.8); hold on;
% %     plot(summary.d_mm, summary.RMS_ey_mm, '-o', 'LineWidth', 1.8);
% %     xlabel('Separation d (mm)');
% %     ylabel('RMS error (mm)');
% %     title('Longitudinal / Lateral RMS Error');
% %     legend('RMS e_x', 'RMS e_y', 'Location', 'best');
% %     grid on;
% % 
% %     nexttile;
% %     plot(summary.d_mm, summary.RMS_gap_mm, '-o', 'LineWidth', 1.8);
% %     xlabel('Separation d (mm)');
% %     ylabel('RMS gap error (mm)');
% %     title('RMS Gap Error vs Separation');
% %     grid on;
% % 
% %     nexttile;
% %     plot(summary.d_mm, summary.Leader_MAX_Lateral_mm, '-o', 'LineWidth', 1.8); hold on;
% %     plot(summary.d_mm, summary.Follower_MAX_Lateral_mm, '-o', 'LineWidth', 1.8);
% %     xlabel('Separation d (mm)');
% %     ylabel('Max lateral deviation (mm)');
% %     title('Straightness vs Separation');
% %     legend('Leader', 'Follower', 'Location', 'best');
% %     grid on;
% % end
% % 
% % %% Optional export
% % % writetable(summary, 'leader_follower_summary.xlsx');
% % 
% % 
% % 
% % % %% Leader-Follower Analysis (End-Aligned Errors, Full Path Plot)
% % % clear; clc; close all;
% % % 
% % % filename = 'leader_follower_tests.xlsx';
% % % dt_ms = 50;
% % % lostTimeout_ms = 200;
% % % moveThresh_mm = 2.0;
% % % 
% % % [~, sheetNames] = xlsfinfo(filename);
% % % summary = table();
% % % 
% % % for s = 1:numel(sheetNames)
% % %     sheetName = sheetNames{s};
% % %     d_mm = str2double(sheetName);
% % % 
% % %     if isnan(d_mm)
% % %         continue;
% % %     end
% % % 
% % %     raw = readcell(filename, 'Sheet', sheetName);
% % % 
% % %     keepRows = ~all(cellfun(@(x) isempty(x) || (isstring(x) && strlength(x)==0), raw), 2);
% % %     raw = raw(keepRows, :);
% % % 
% % %     leader = raw(:,1:5);
% % %     follower = raw(:,7:11);
% % % 
% % %     leaderKeep = cellfun(@isnumeric, leader(:,2));
% % %     followerKeep = cellfun(@isnumeric, follower(:,2));
% % % 
% % %     leader = leader(leaderKeep,:);
% % %     follower = follower(followerKeep,:);
% % % 
% % %     if isempty(leader) || isempty(follower)
% % %         continue;
% % %     end
% % % 
% % %     L = cell2mat(leader);
% % %     F = cell2mat(follower);
% % % 
% % %     %% Raw columns
% % %     L_time = L(:,2);
% % %     L_x = -L(:,3);      % flip leader x internally
% % %     L_y = L(:,4);
% % %     L_th = L(:,5);
% % % 
% % %     F_time = F(:,2);
% % %     F_x = F(:,3);
% % %     F_y = F(:,4);
% % %     F_th = F(:,5);
% % % 
% % %     %% Full trajectories for path plot only
% % %     L_x_plot = L_x - L_x(1);
% % %     L_y_plot = L_y - L_y(1);
% % % 
% % %     F_x_plot = F_x - F_x(1);
% % %     F_y_plot = F_y - F_y(1);
% % % 
% % %     %% Movement start detection
% % %     L_start_idx = 1;   % leader start is first sample by definition
% % % 
% % %     F_start_idx = find(abs(F_x - F_x(1)) > moveThresh_mm | ...
% % %                        abs(F_y - F_y(1)) > moveThresh_mm, 1, 'first');
% % %     if isempty(F_start_idx), F_start_idx = 1; end
% % % 
% % %     leader_start_time_ms = 0;
% % %     follower_start_time_ms = F_time(F_start_idx) - L_time(1);
% % %     start_delay_ms = follower_start_time_ms;
% % %     start_delay_samples = round(start_delay_ms / dt_ms);
% % % 
% % %     %% End alignment for error calculation
% % %     L_stop_ref = L_time(end);
% % %     F_stop_ref = F_time(end) - lostTimeout_ms;
% % % 
% % %     L_tstop = L_time - L_stop_ref;
% % %     F_tstop = F_time - F_stop_ref;
% % % 
% % %     tStart = min(L_tstop);
% % %     tCommon = (tStart:dt_ms:0)';
% % % 
% % %     % Leader interpolated on comparison grid
% % %     L_xi = interp1(L_tstop, L_x_plot, tCommon, 'linear', 'extrap');
% % %     L_yi = interp1(L_tstop, L_y_plot, tCommon, 'linear', 'extrap');
% % % 
% % %     % Follower interpolated on comparison grid
% % %     F_xi = interp1(F_tstop, F_x_plot, tCommon, 'linear', NaN);
% % %     F_yi = interp1(F_tstop, F_y_plot, tCommon, 'linear', NaN);
% % % 
% % %     % Fill missing pre-motion follower values with 0
% % %     F_xi(isnan(F_xi)) = 0;
% % %     F_yi(isnan(F_yi)) = 0;
% % % 
% % %     %% Errors over aligned time
% % %     ex = F_xi - L_xi;
% % %     ey = F_yi - L_yi;
% % %     etotal = sqrt(ex.^2 + ey.^2);
% % %     gapErr = (L_xi - F_xi) - d_mm;
% % % 
% % %     RMS_ex = sqrt(mean(ex.^2));
% % %     RMS_ey = sqrt(mean(ey.^2));
% % %     RMS_etotal = sqrt(mean(etotal.^2));
% % %     MAX_etotal = max(etotal);
% % % 
% % %     %% Plots
% % %     figure('Name', sprintf('d = %g mm', d_mm), 'Color', 'w', ...
% % %         'Position', [80 80 1400 900]);
% % %     tiledlayout(3,2);
% % % 
% % %     % Full path plot
% % %     nexttile;
% % %     plot(L_x_plot, L_y_plot, 'LineWidth', 1.8); hold on;
% % %     plot(F_x_plot, F_y_plot, 'LineWidth', 1.8);
% % %     xlabel('x (mm)');
% % %     ylabel('y (mm)');
% % %     title(sprintf('Full Paths, d = %g mm', d_mm));
% % %     legend('Leader', 'Follower', 'Location', 'best');
% % %     axis equal;
% % %     grid on;
% % % 
% % %     % Total path error over time
% % %     nexttile;
% % %     plot(tCommon, etotal, 'k', 'LineWidth', 1.8);
% % %     xlabel('Time relative to leader stop (ms)');
% % %     ylabel('Total path error (mm)');
% % %     title('Total Path Error vs Time');
% % %     grid on;
% % % 
% % %     % Longitudinal error
% % %     nexttile;
% % %     plot(tCommon, ex, 'LineWidth', 1.8);
% % %     xlabel('Time relative to leader stop (ms)');
% % %     ylabel('Longitudinal error e_x (mm)');
% % %     title('Longitudinal Error vs Time');
% % %     grid on;
% % % 
% % %     % Lateral error
% % %     nexttile;
% % %     plot(tCommon, ey, 'LineWidth', 1.8);
% % %     xlabel('Time relative to leader stop (ms)');
% % %     ylabel('Lateral error e_y (mm)');
% % %     title('Lateral Error vs Time');
% % %     grid on;
% % % 
% % %     % Gap error
% % %     nexttile;
% % %     plot(tCommon, gapErr, 'LineWidth', 1.8);
% % %     xlabel('Time relative to leader stop (ms)');
% % %     ylabel('Gap error (mm)');
% % %     title(sprintf('Gap Error, d = %g mm', d_mm));
% % %     grid on;
% % % 
% % %     % Text summary
% % %     nexttile;
% % %     axis off;
% % %     text(0.02, 0.85, sprintf('Leader start = %.0f ms', leader_start_time_ms), 'FontSize', 11);
% % %     text(0.02, 0.72, sprintf('Follower start = %.0f ms', follower_start_time_ms), 'FontSize', 11);
% % %     text(0.02, 0.59, sprintf('Follower delay = %.0f ms (%d samples)', ...
% % %         start_delay_ms, start_delay_samples), 'FontSize', 11);
% % %     text(0.02, 0.43, sprintf('RMS e_x = %.2f mm', RMS_ex), 'FontSize', 11);
% % %     text(0.02, 0.31, sprintf('RMS e_y = %.2f mm', RMS_ey), 'FontSize', 11);
% % %     text(0.02, 0.19, sprintf('RMS e_total = %.2f mm', RMS_etotal), 'FontSize', 11);
% % %     text(0.02, 0.07, sprintf('MAX e_total = %.2f mm', MAX_etotal), 'FontSize', 11);
% % % 
% % %     sgtitle(sprintf('Leader-Follower Analysis, d = %g mm', d_mm));
% % % 
% % %     summary = [summary; table(d_mm, leader_start_time_ms, follower_start_time_ms, ...
% % %         start_delay_ms, start_delay_samples, RMS_ex, RMS_ey, RMS_etotal, MAX_etotal)];
% % % end
% % % 
% % % disp(summary);
% % % 
% % % 
% % % 
% % % % %% Leader-Follower Analysis (End-Aligned, Follower Zero-Padded)
% % % % clear; clc; close all;
% % % % 
% % % % %% ---------------- USER SETTINGS ----------------
% % % % filename = 'leader_follower_tests.xlsx';   % change this
% % % % dt_ms = 50;                                % sample interval in your logs
% % % % lostTimeout_ms = 200;                      % follower LOST_TIMEOUT_MS
% % % % moveThresh_mm = 2.0;                       % movement threshold
% % % % savePlots = false;
% % % % plotFolder = 'matlab_plots_end_aligned';
% % % % 
% % % % if savePlots && ~exist(plotFolder, 'dir')
% % % %     mkdir(plotFolder);
% % % % end
% % % % 
% % % % %% ---------------- LOAD SHEETS ----------------
% % % % [~, sheetNames] = xlsfinfo(filename);
% % % % if isempty(sheetNames)
% % % %     error('No sheets found in workbook.');
% % % % end
% % % % 
% % % % summary = table();
% % % % 
% % % % for s = 1:numel(sheetNames)
% % % %     sheetName = sheetNames{s};
% % % %     d_mm = str2double(sheetName);
% % % % 
% % % %     if isnan(d_mm)
% % % %         warning('Skipping sheet "%s" because name is not numeric.', sheetName);
% % % %         continue;
% % % %     end
% % % % 
% % % %     raw = readcell(filename, 'Sheet', sheetName);
% % % % 
% % % %     % Drop empty rows
% % % %     keepRows = ~all(cellfun(@(x) isempty(x) || (isstring(x) && strlength(x)==0), raw), 2);
% % % %     raw = raw(keepRows, :);
% % % % 
% % % %     if size(raw,2) < 11
% % % %         warning('Skipping sheet "%s": expected at least 11 columns.', sheetName);
% % % %         continue;
% % % %     end
% % % % 
% % % %     % Leader A:E
% % % %     leader = raw(:,1:5);
% % % %     % Follower G:K
% % % %     follower = raw(:,7:11);
% % % % 
% % % %     leaderKeep = cellfun(@isnumeric, leader(:,2));
% % % %     followerKeep = cellfun(@isnumeric, follower(:,2));
% % % % 
% % % %     leader = leader(leaderKeep,:);
% % % %     follower = follower(followerKeep,:);
% % % % 
% % % %     if isempty(leader) || isempty(follower)
% % % %         warning('Skipping sheet "%s": no usable leader/follower data.', sheetName);
% % % %         continue;
% % % %     end
% % % % 
% % % %     L = cell2mat(leader);
% % % %     F = cell2mat(follower);
% % % % 
% % % %     %% ---------------- EXTRACT COLUMNS ----------------
% % % %     % Leader raw
% % % %     L_time = L(:,2);
% % % %     L_x_raw = L(:,3);
% % % %     L_y = L(:,4);
% % % %     L_th = L(:,5);
% % % % 
% % % %     % Flip leader x so reverse motion becomes positive path direction
% % % %     L_x = -L_x_raw;
% % % % 
% % % %     % Follower
% % % %     F_time = F(:,2);
% % % %     F_x = F(:,3);
% % % %     F_y = F(:,4);
% % % %     F_th = F(:,5);
% % % % 
% % % %     %% ---------------- FIND MOVEMENT START ----------------
% % % %     L_start_idx = find(abs(L_x - L_x(1)) > moveThresh_mm | ...
% % % %                        abs(L_y - L_y(1)) > moveThresh_mm, 1, 'first');
% % % %     F_start_idx = find(abs(F_x - F_x(1)) > moveThresh_mm | ...
% % % %                        abs(F_y - F_y(1)) > moveThresh_mm, 1, 'first');
% % % % 
% % % %     if isempty(L_start_idx), L_start_idx = 1; end
% % % %     if isempty(F_start_idx), F_start_idx = 1; end
% % % % 
% % % %     leader_start_time_ms = L_time(L_start_idx);
% % % %     follower_start_time_ms = F_time(F_start_idx);
% % % % 
% % % %     start_delay_ms = follower_start_time_ms - leader_start_time_ms;
% % % %     start_delay_samples = round(start_delay_ms / dt_ms);
% % % % 
% % % %     %% ---------------- END ALIGNMENT ----------------
% % % %     % Leader stop event = leader final timestamp
% % % %     % Follower stop-reference = follower final timestamp - lost timeout
% % % %     L_stop_ref = L_time(end);
% % % %     F_stop_ref = F_time(end) - lostTimeout_ms;
% % % % 
% % % %     % Relative-to-stop time
% % % %     L_tstop = L_time - L_stop_ref;      % ends at 0
% % % %     F_tstop = F_time - F_stop_ref;      % corrected to estimated beacon-off event
% % % % 
% % % %     %% ---------------- COMMON END-ALIGNED TIME GRID ----------------
% % % %     % Start as far back as leader data goes
% % % %     tStart = min(L_tstop);   % negative
% % % %     tCommon = (tStart:dt_ms:0)';
% % % % 
% % % %     % Interpolate leader on this full grid
% % % %     L_xi = interp1(L_tstop, L_x, tCommon, 'linear', 'extrap');
% % % %     L_yi = interp1(L_tstop, L_y, tCommon, 'linear', 'extrap');
% % % %     L_thi = interp1(L_tstop, L_th, tCommon, 'linear', 'extrap');
% % % % 
% % % %     % Interpolate follower only where follower exists
% % % %     F_xi = interp1(F_tstop, F_x, tCommon, 'linear', NaN);
% % % %     F_yi = interp1(F_tstop, F_y, tCommon, 'linear', NaN);
% % % %     F_thi = interp1(F_tstop, F_th, tCommon, 'linear', NaN);
% % % % 
% % % %     % Replace missing follower samples before it starts with 0,0,0
% % % %     F_xi(isnan(F_xi)) = 0;
% % % %     F_yi(isnan(F_yi)) = 0;
% % % %     F_thi(isnan(F_thi)) = 0;
% % % % 
% % % %     %% ---------------- PATH ERRORS ----------------
% % % %     % Longitudinal error: x difference
% % % %     ex = F_xi - L_xi;
% % % % 
% % % %     % Lateral error: y difference
% % % %     ey = F_yi - L_yi;
% % % % 
% % % %     % Total path error
% % % %     etotal = sqrt(ex.^2 + ey.^2);
% % % % 
% % % %     % Gap error relative to desired separation d
% % % %     gapErr = (L_xi - F_xi) - d_mm;
% % % % 
% % % %     %% ---------------- METRICS ----------------
% % % %     RMS_ex = sqrt(mean(ex.^2));
% % % %     RMS_ey = sqrt(mean(ey.^2));
% % % %     RMS_etotal = sqrt(mean(etotal.^2));
% % % % 
% % % %     MAX_ex = max(abs(ex));
% % % %     MAX_ey = max(abs(ey));
% % % %     MAX_etotal = max(etotal);
% % % % 
% % % %     RMS_gap = sqrt(mean(gapErr.^2));
% % % %     MAX_gap = max(abs(gapErr));
% % % % 
% % % %     leader_max_lat = max(abs(L_yi));
% % % %     follower_max_lat = max(abs(F_yi));
% % % % 
% % % %     leader_rms_lat = sqrt(mean(L_yi.^2));
% % % %     follower_rms_lat = sqrt(mean(F_yi.^2));
% % % % 
% % % %     %% ---------------- PLOTS ----------------
% % % %     figure('Name', sprintf('d = %g mm', d_mm), 'Color', 'w', ...
% % % %         'Position', [80 80 1400 900]);
% % % %     tiledlayout(3,2);
% % % % 
% % % %     % 1. End-aligned full paths
% % % %     nexttile;
% % % %     plot(L_xi, L_yi, 'LineWidth', 1.8); hold on;
% % % %     plot(F_xi, F_yi, 'LineWidth', 1.8);
% % % %     xlabel('x (mm)');
% % % %     ylabel('y (mm)');
% % % %     title(sprintf('End-Aligned Paths, d = %g mm', d_mm));
% % % %     legend('Leader', 'Follower', 'Location', 'best');
% % % %     axis equal;
% % % %     grid on;
% % % % 
% % % %     % 2. Total path error over time
% % % %     nexttile;
% % % %     plot(tCommon, etotal, 'k', 'LineWidth', 1.8);
% % % %     xlabel('Time relative to stop (ms)');
% % % %     ylabel('Total path error (mm)');
% % % %     title('Total Path Error vs Time');
% % % %     grid on;
% % % % 
% % % %     % 3. Longitudinal error
% % % %     nexttile;
% % % %     plot(tCommon, ex, 'LineWidth', 1.8);
% % % %     xlabel('Time relative to stop (ms)');
% % % %     ylabel('Longitudinal error e_x (mm)');
% % % %     title('Longitudinal Error vs Time');
% % % %     grid on;
% % % % 
% % % %     % 4. Lateral error
% % % %     nexttile;
% % % %     plot(tCommon, ey, 'LineWidth', 1.8);
% % % %     xlabel('Time relative to stop (ms)');
% % % %     ylabel('Lateral error e_y (mm)');
% % % %     title('Lateral Error vs Time');
% % % %     grid on;
% % % % 
% % % %     % 5. Gap error
% % % %     nexttile;
% % % %     plot(tCommon, gapErr, 'LineWidth', 1.8);
% % % %     xlabel('Time relative to stop (ms)');
% % % %     ylabel('Gap error (mm)');
% % % %     title(sprintf('Gap Error (desired d = %g mm)', d_mm));
% % % %     grid on;
% % % % 
% % % %     % 6. Text summary
% % % %     nexttile;
% % % %     axis off;
% % % %     text(0.02, 0.90, sprintf('Leader start time: %.0f ms', leader_start_time_ms), 'FontSize', 11);
% % % %     text(0.02, 0.78, sprintf('Follower start time: %.0f ms', follower_start_time_ms), 'FontSize', 11);
% % % %     text(0.02, 0.66, sprintf('Follower delay: %.0f ms (%d samples)', ...
% % % %         start_delay_ms, start_delay_samples), 'FontSize', 11);
% % % %     text(0.02, 0.52, sprintf('RMS e_x: %.2f mm', RMS_ex), 'FontSize', 11);
% % % %     text(0.02, 0.42, sprintf('RMS e_y: %.2f mm', RMS_ey), 'FontSize', 11);
% % % %     text(0.02, 0.32, sprintf('RMS e_{total}: %.2f mm', RMS_etotal), 'FontSize', 11);
% % % %     text(0.02, 0.20, sprintf('MAX e_{total}: %.2f mm', MAX_etotal), 'FontSize', 11);
% % % % 
% % % %     sgtitle(sprintf('Leader-Follower Analysis, d = %g mm', d_mm));
% % % % 
% % % %     if savePlots
% % % %         saveas(gcf, fullfile(plotFolder, sprintf('end_aligned_d_%g.png', d_mm)));
% % % %     end
% % % % 
% % % %     %% ---------------- STORE SUMMARY ----------------
% % % %     summary = [summary; table( ...
% % % %         d_mm, ...
% % % %         leader_start_time_ms, ...
% % % %         follower_start_time_ms, ...
% % % %         start_delay_ms, ...
% % % %         start_delay_samples, ...
% % % %         RMS_ex, RMS_ey, RMS_etotal, ...
% % % %         MAX_ex, MAX_ey, MAX_etotal, ...
% % % %         RMS_gap, MAX_gap, ...
% % % %         leader_rms_lat, leader_max_lat, ...
% % % %         follower_rms_lat, follower_max_lat, ...
% % % %         'VariableNames', { ...
% % % %         'd_mm', ...
% % % %         'LeaderStart_ms', ...
% % % %         'FollowerStart_ms', ...
% % % %         'FollowerDelay_ms', ...
% % % %         'FollowerDelay_samples', ...
% % % %         'RMS_ex_mm', 'RMS_ey_mm', 'RMS_etotal_mm', ...
% % % %         'MAX_ex_mm', 'MAX_ey_mm', 'MAX_etotal_mm', ...
% % % %         'RMS_gap_mm', 'MAX_gap_mm', ...
% % % %         'Leader_RMS_Lateral_mm', 'Leader_MAX_Lateral_mm', ...
% % % %         'Follower_RMS_Lateral_mm', 'Follower_MAX_Lateral_mm'})];
% % % % end
% % % % 
% % % % %% ---------------- SUMMARY PLOTS ----------------
% % % % if ~isempty(summary)
% % % %     summary = sortrows(summary, 'd_mm');
% % % %     disp(summary);
% % % % 
% % % %     figure('Name', 'Summary Across Distances', 'Color', 'w', ...
% % % %         'Position', [120 120 1400 800]);
% % % %     tiledlayout(2,3);
% % % % 
% % % %     nexttile;
% % % %     plot(summary.d_mm, summary.FollowerDelay_ms, '-o', 'LineWidth', 1.8);
% % % %     xlabel('Separation d (mm)');
% % % %     ylabel('Follower delay (ms)');
% % % %     title('Follower Start Delay vs Separation');
% % % %     grid on;
% % % % 
% % % %     nexttile;
% % % %     plot(summary.d_mm, summary.RMS_etotal_mm, '-o', 'LineWidth', 1.8);
% % % %     xlabel('Separation d (mm)');
% % % %     ylabel('RMS total path error (mm)');
% % % %     title('RMS Total Path Error vs Separation');
% % % %     grid on;
% % % % 
% % % %     nexttile;
% % % %     plot(summary.d_mm, summary.MAX_etotal_mm, '-o', 'LineWidth', 1.8);
% % % %     xlabel('Separation d (mm)');
% % % %     ylabel('Max total path error (mm)');
% % % %     title('Max Total Path Error vs Separation');
% % % %     grid on;
% % % % 
% % % %     nexttile;
% % % %     plot(summary.d_mm, summary.RMS_ex_mm, '-o', 'LineWidth', 1.8); hold on;
% % % %     plot(summary.d_mm, summary.RMS_ey_mm, '-o', 'LineWidth', 1.8);
% % % %     xlabel('Separation d (mm)');
% % % %     ylabel('RMS error (mm)');
% % % %     title('Longitudinal / Lateral RMS Error');
% % % %     legend('RMS e_x', 'RMS e_y', 'Location', 'best');
% % % %     grid on;
% % % % 
% % % %     nexttile;
% % % %     plot(summary.d_mm, summary.RMS_gap_mm, '-o', 'LineWidth', 1.8);
% % % %     xlabel('Separation d (mm)');
% % % %     ylabel('RMS gap error (mm)');
% % % %     title('RMS Gap Error vs Separation');
% % % %     grid on;
% % % % 
% % % %     nexttile;
% % % %     plot(summary.d_mm, summary.Leader_MAX_Lateral_mm, '-o', 'LineWidth', 1.8); hold on;
% % % %     plot(summary.d_mm, summary.Follower_MAX_Lateral_mm, '-o', 'LineWidth', 1.8);
% % % %     xlabel('Separation d (mm)');
% % % %     ylabel('Max lateral deviation (mm)');
% % % %     title('Straightness vs Separation');
% % % %     legend('Leader', 'Follower', 'Location', 'best');
% % % %     grid on;
% % % % end
% % % 
% % % %% Optional export
% % % % writetable(summary, 'leader_follower_summary.xlsx');
% % % 
% % % 
% % % 
% % % % %% Leader-Follower Comparison Across Excel Sheets
% % % % % Each sheet name is the separation distance d in mm, e.g. "50", "100", "150".
% % % % % Expected columns per sheet:
% % % % %   A: leader i
% % % % %   B: leader time_ms
% % % % %   C: leader x_mm   (raw, likely negative for reverse)
% % % % %   D: leader y_mm
% % % % %   E: leader theta_rad
% % % % %   F: blank/separator optional
% % % % %   G: follower i
% % % % %   H: follower time_ms
% % % % %   I: follower x_mm
% % % % %   J: follower y_mm
% % % % %   K: follower theta_rad
% % % % %
% % % % % This script flips leader x internally so reverse travel becomes positive-x.
% % % % 
% % % % clear; clc; close all;
% % % % 
% % % % %% ---------------- USER SETTINGS ----------------
% % % % filename = 'leader_follower_tests.xlsx';   % <-- change this
% % % % 
% % % % dt_ms = 50;                     % sample grid used for interpolation
% % % % startMoveThresholdMM = 2.0;     % movement start detection threshold
% % % % savePlots = false;
% % % % plotFolder = 'matlab_plots';
% % % % 
% % % % if savePlots && ~exist(plotFolder, 'dir')
% % % %     mkdir(plotFolder);
% % % % end
% % % % 
% % % % %% ---------------- GET SHEETS ----------------
% % % % [~, sheetNames] = xlsfinfo(filename);
% % % % if isempty(sheetNames)
% % % %     error('No sheets found in workbook: %s', filename);
% % % % end
% % % % 
% % % % summary = table();
% % % % allResults = struct();
% % % % 
% % % % %% ---------------- PROCESS SHEETS ----------------
% % % % for s = 1:numel(sheetNames)
% % % %     sheetName = sheetNames{s};
% % % % 
% % % %     d_mm = str2double(sheetName);
% % % %     if isnan(d_mm)
% % % %         warning('Skipping sheet "%s" because sheet name is not numeric.', sheetName);
% % % %         continue;
% % % %     end
% % % % 
% % % %     raw = readcell(filename, 'Sheet', sheetName);
% % % % 
% % % %     % Remove empty rows
% % % %     keepRows = ~all(cellfun(@(x) isempty(x) || (isstring(x) && strlength(x)==0), raw), 2);
% % % %     raw = raw(keepRows, :);
% % % % 
% % % %     if size(raw,2) < 11
% % % %         warning('Skipping sheet "%s": expected at least 11 columns.', sheetName);
% % % %         continue;
% % % %     end
% % % % 
% % % %     % Leader block A:E
% % % %     leader = raw(:,1:5);
% % % %     % Follower block G:K
% % % %     follower = raw(:,7:11);
% % % % 
% % % %     leaderKeep = cellfun(@isnumeric, leader(:,2));
% % % %     followerKeep = cellfun(@isnumeric, follower(:,2));
% % % % 
% % % %     leader = leader(leaderKeep,:);
% % % %     follower = follower(followerKeep,:);
% % % % 
% % % %     if isempty(leader) || isempty(follower)
% % % %         warning('Skipping sheet "%s": empty leader or follower data.', sheetName);
% % % %         continue;
% % % %     end
% % % % 
% % % %     L = cell2mat(leader);
% % % %     F = cell2mat(follower);
% % % % 
% % % %     % Leader columns
% % % %     L_time = L(:,2);
% % % %     L_x_raw = L(:,3);
% % % %     L_y = L(:,4);
% % % %     L_th = L(:,5);
% % % % 
% % % %     % Flip leader x internally so reverse motion becomes positive travel
% % % %     L_x = -L_x_raw;
% % % % 
% % % %     % Follower columns
% % % %     F_time = F(:,2);
% % % %     F_x = F(:,3);
% % % %     F_y = F(:,4);
% % % %     F_th = F(:,5);
% % % % 
% % % %     %% ---- ALIGN EACH RUN BY ITS OWN MOVEMENT START ----
% % % %     L_start_idx = find(abs(L_x - L_x(1)) > startMoveThresholdMM | ...
% % % %                        abs(L_y - L_y(1)) > startMoveThresholdMM, 1, 'first');
% % % %     F_start_idx = find(abs(F_x - F_x(1)) > startMoveThresholdMM | ...
% % % %                        abs(F_y - F_y(1)) > startMoveThresholdMM, 1, 'first');
% % % % 
% % % %     if isempty(L_start_idx), L_start_idx = 1; end
% % % %     if isempty(F_start_idx), F_start_idx = 1; end
% % % % 
% % % %     L_trel = L_time - L_time(L_start_idx);
% % % %     F_trel = F_time - F_time(F_start_idx);
% % % % 
% % % %     L_xrel = L_x - L_x(L_start_idx);
% % % %     L_yrel = L_y - L_y(L_start_idx);
% % % % 
% % % %     F_xrel = F_x - F_x(F_start_idx);
% % % %     F_yrel = F_y - F_y(F_start_idx);
% % % % 
% % % %     %% ---- COMMON TIME GRID ----
% % % %     tEnd = min(max(L_trel), max(F_trel));
% % % %     if tEnd <= 0
% % % %         warning('Skipping sheet "%s": no positive common time range.', sheetName);
% % % %         continue;
% % % %     end
% % % % 
% % % %     tCommon = (0:dt_ms:tEnd)';
% % % % 
% % % %     L_xi = interp1(L_trel, L_xrel, tCommon, 'linear', 'extrap');
% % % %     L_yi = interp1(L_trel, L_yrel, tCommon, 'linear', 'extrap');
% % % %     L_thi = interp1(L_trel, L_th,   tCommon, 'linear', 'extrap');
% % % % 
% % % %     F_xi = interp1(F_trel, F_xrel, tCommon, 'linear', 'extrap');
% % % %     F_yi = interp1(F_trel, F_yrel, tCommon, 'linear', 'extrap');
% % % %     F_thi = interp1(F_trel, F_th,   tCommon, 'linear', 'extrap');
% % % % 
% % % %     %% ---- ERROR METRICS ----
% % % %     ex = F_xi - L_xi;
% % % %     ey = F_yi - L_yi;
% % % %     etotal = sqrt(ex.^2 + ey.^2);
% % % % 
% % % %     % Gap error relative to desired separation d
% % % %     gapErr = (L_xi - F_xi) - d_mm;
% % % % 
% % % %     leaderLateral = L_yi;
% % % %     followerLateral = F_yi;
% % % % 
% % % %     RMS_ex = sqrt(mean(ex.^2));
% % % %     RMS_ey = sqrt(mean(ey.^2));
% % % %     RMS_etotal = sqrt(mean(etotal.^2));
% % % %     MAX_etotal = max(etotal);
% % % % 
% % % %     RMS_gap = sqrt(mean(gapErr.^2));
% % % %     MAX_gap = max(abs(gapErr));
% % % % 
% % % %     leader_rms_lat = sqrt(mean(leaderLateral.^2));
% % % %     leader_max_lat = max(abs(leaderLateral));
% % % %     follower_rms_lat = sqrt(mean(followerLateral.^2));
% % % %     follower_max_lat = max(abs(followerLateral));
% % % % 
% % % %     %% ---- LAG ESTIMATION ----
% % % %     lag_xcorr_ms = estimate_lag_trimmed_ms(L_trel, L_xrel, F_trel, F_xrel, dt_ms);
% % % % 
% % % %     tL10 = time_to_percent(L_trel, L_xrel, 0.10);
% % % %     tF10 = time_to_percent(F_trel, F_xrel, 0.10);
% % % %     tL50 = time_to_percent(L_trel, L_xrel, 0.50);
% % % %     tF50 = time_to_percent(F_trel, F_xrel, 0.50);
% % % %     tL90 = time_to_percent(L_trel, L_xrel, 0.90);
% % % %     tF90 = time_to_percent(F_trel, F_xrel, 0.90);
% % % % 
% % % %     lag10_ms = tF10 - tL10;
% % % %     lag50_ms = tF50 - tL50;
% % % %     lag90_ms = tF90 - tL90;
% % % % 
% % % %     %% ---- STORE RESULTS ----
% % % %     result = struct();
% % % %     result.d_mm = d_mm;
% % % %     result.t = tCommon;
% % % %     result.L_x = L_xi;
% % % %     result.L_y = L_yi;
% % % %     result.F_x = F_xi;
% % % %     result.F_y = F_yi;
% % % %     result.ex = ex;
% % % %     result.ey = ey;
% % % %     result.etotal = etotal;
% % % %     result.gapErr = gapErr;
% % % %     result.lag_xcorr_ms = lag_xcorr_ms;
% % % %     result.lag10_ms = lag10_ms;
% % % %     result.lag50_ms = lag50_ms;
% % % %     result.lag90_ms = lag90_ms;
% % % % 
% % % %     allResults.(matlab.lang.makeValidName(sprintf('d_%g', d_mm))) = result;
% % % % 
% % % %     summary = [summary; table( ...
% % % %         d_mm, ...
% % % %         RMS_ex, RMS_ey, RMS_etotal, MAX_etotal, ...
% % % %         RMS_gap, MAX_gap, ...
% % % %         leader_rms_lat, leader_max_lat, ...
% % % %         follower_rms_lat, follower_max_lat, ...
% % % %         lag_xcorr_ms, lag10_ms, lag50_ms, lag90_ms, ...
% % % %         'VariableNames', { ...
% % % %         'd_mm', ...
% % % %         'RMS_ex_mm', 'RMS_ey_mm', 'RMS_etotal_mm', 'MAX_etotal_mm', ...
% % % %         'RMS_gap_mm', 'MAX_gap_mm', ...
% % % %         'Leader_RMS_Lateral_mm', 'Leader_MAX_Lateral_mm', ...
% % % %         'Follower_RMS_Lateral_mm', 'Follower_MAX_Lateral_mm', ...
% % % %         'Lag_XCorr_ms', 'Lag10_ms', 'Lag50_ms', 'Lag90_ms'})];
% % % % 
% % % %     %% ---- PLOTS FOR THIS SHEET ----
% % % %     figure('Name', sprintf('Separation %g mm', d_mm), 'Color', 'w', 'Position', [100 100 1400 900]);
% % % %     tiledlayout(3,2);
% % % % 
% % % %     % Paths
% % % %     nexttile;
% % % %     plot(L_xi, L_yi, 'LineWidth', 1.8); hold on;
% % % %     plot(F_xi, F_yi, 'LineWidth', 1.8);
% % % %     xlabel('x (mm)');
% % % %     ylabel('y (mm)');
% % % %     title(sprintf('Paths, d = %g mm', d_mm));
% % % %     legend('Leader', 'Follower', 'Location', 'best');
% % % %     axis equal;
% % % %     grid on;
% % % % 
% % % %     % Tracking errors
% % % %     nexttile;
% % % %     plot(tCommon, ex, 'LineWidth', 1.4); hold on;
% % % %     plot(tCommon, ey, 'LineWidth', 1.4);
% % % %     plot(tCommon, etotal, 'k', 'LineWidth', 1.8);
% % % %     xlabel('Time (ms)');
% % % %     ylabel('Error (mm)');
% % % %     title('Tracking Error vs Time');
% % % %     legend('e_x', 'e_y', 'e_{total}', 'Location', 'best');
% % % %     grid on;
% % % % 
% % % %     % Gap error
% % % %     nexttile;
% % % %     plot(tCommon, gapErr, 'LineWidth', 1.8);
% % % %     xlabel('Time (ms)');
% % % %     ylabel('Gap error (mm)');
% % % %     title(sprintf('Gap Error (desired d = %g mm)', d_mm));
% % % %     grid on;
% % % % 
% % % %     % Lateral deviation
% % % %     nexttile;
% % % %     plot(tCommon, leaderLateral, 'LineWidth', 1.8); hold on;
% % % %     plot(tCommon, followerLateral, 'LineWidth', 1.8);
% % % %     xlabel('Time (ms)');
% % % %     ylabel('y (mm)');
% % % %     title('Lateral Deviation');
% % % %     legend('Leader y', 'Follower y', 'Location', 'best');
% % % %     grid on;
% % % % 
% % % %     % x vs time
% % % %     nexttile;
% % % %     plot(tCommon, L_xi, 'LineWidth', 1.8); hold on;
% % % %     plot(tCommon, F_xi, 'LineWidth', 1.8);
% % % %     xlabel('Time (ms)');
% % % %     ylabel('x (mm)');
% % % %     title('Longitudinal Motion');
% % % %     legend('Leader x', 'Follower x', 'Location', 'best');
% % % %     grid on;
% % % % 
% % % %     % Lag text panel
% % % %     nexttile;
% % % %     axis off;
% % % %     text(0.05, 0.85, sprintf('Lag (xcorr): %.1f ms', lag_xcorr_ms), 'FontSize', 12);
% % % %     text(0.05, 0.70, sprintf('Lag 10%% travel: %.1f ms', lag10_ms), 'FontSize', 12);
% % % %     text(0.05, 0.55, sprintf('Lag 50%% travel: %.1f ms', lag50_ms), 'FontSize', 12);
% % % %     text(0.05, 0.40, sprintf('Lag 90%% travel: %.1f ms', lag90_ms), 'FontSize', 12);
% % % %     text(0.05, 0.20, sprintf('RMS total error: %.2f mm', RMS_etotal), 'FontSize', 12);
% % % % 
% % % %     sgtitle(sprintf('Leader-Follower Comparison, d = %g mm', d_mm));
% % % % 
% % % %     if savePlots
% % % %         saveas(gcf, fullfile(plotFolder, sprintf('comparison_d_%g.png', d_mm)));
% % % %     end
% % % % end
% % % % 
% % % % %% ---------------- SUMMARY PLOTS ----------------
% % % % if ~isempty(summary)
% % % %     summary = sortrows(summary, 'd_mm');
% % % % 
% % % %     disp('Summary metrics across all separation distances:');
% % % %     disp(summary);
% % % % 
% % % %     figure('Name', 'Summary Across Distances', 'Color', 'w', 'Position', [150 150 1400 800]);
% % % %     tiledlayout(2,3);
% % % % 
% % % %     nexttile;
% % % %     plot(summary.d_mm, summary.RMS_etotal_mm, '-o', 'LineWidth', 1.8);
% % % %     xlabel('Separation d (mm)');
% % % %     ylabel('RMS total error (mm)');
% % % %     title('RMS Total Error vs Separation');
% % % %     grid on;
% % % % 
% % % %     nexttile;
% % % %     plot(summary.d_mm, summary.MAX_etotal_mm, '-o', 'LineWidth', 1.8);
% % % %     xlabel('Separation d (mm)');
% % % %     ylabel('Max total error (mm)');
% % % %     title('Max Total Error vs Separation');
% % % %     grid on;
% % % % 
% % % %     nexttile;
% % % %     plot(summary.d_mm, summary.RMS_gap_mm, '-o', 'LineWidth', 1.8);
% % % %     xlabel('Separation d (mm)');
% % % %     ylabel('RMS gap error (mm)');
% % % %     title('RMS Gap Error vs Separation');
% % % %     grid on;
% % % % 
% % % %     nexttile;
% % % %     plot(summary.d_mm, summary.Leader_MAX_Lateral_mm, '-o', 'LineWidth', 1.8); hold on;
% % % %     plot(summary.d_mm, summary.Follower_MAX_Lateral_mm, '-o', 'LineWidth', 1.8);
% % % %     xlabel('Separation d (mm)');
% % % %     ylabel('Max lateral deviation (mm)');
% % % %     title('Straightness vs Separation');
% % % %     legend('Leader', 'Follower', 'Location', 'best');
% % % %     grid on;
% % % % 
% % % %     nexttile;
% % % %     plot(summary.d_mm, summary.Lag_XCorr_ms, '-o', 'LineWidth', 1.8); hold on;
% % % %     plot(summary.d_mm, summary.Lag50_ms, '-o', 'LineWidth', 1.8);
% % % %     xlabel('Separation d (mm)');
% % % %     ylabel('Lag (ms)');
% % % %     title('Lag vs Separation');
% % % %     legend('XCorr lag', '50% travel lag', 'Location', 'best');
% % % %     grid on;
% % % % 
% % % %     nexttile;
% % % %     axis off;
% % % %     text(0.05, 0.90, 'Summary variables:', 'FontWeight', 'bold');
% % % %     text(0.05, 0.75, 'RMS_etotal: RMS of sqrt(e_x^2 + e_y^2)');
% % % %     text(0.05, 0.60, 'RMS_gap: RMS of longitudinal gap error');
% % % %     text(0.05, 0.45, 'Lag_XCorr: overall delay from motion cross-correlation');
% % % %     text(0.05, 0.30, 'Lag50: follower delay to reach 50% travel');
% % % % end
% % % % 
% % % % %% ---------------- OPTIONAL: EXPORT SUMMARY ----------------
% % % % % writetable(summary, 'leader_follower_summary.xlsx');
% % % % 
% % % % %% ================= LOCAL FUNCTIONS =================
% % % % 
% % % % function lag_ms = estimate_lag_trimmed_ms(tL, xL, tF, xF, dt_ms)
% % % %     % Common time base
% % % %     tEnd = min(max(tL), max(tF));
% % % %     tCommon = (0:dt_ms:tEnd)';
% % % % 
% % % %     xLi = interp1(tL, xL, tCommon, 'linear', 'extrap');
% % % %     xFi = interp1(tF, xF, tCommon, 'linear', 'extrap');
% % % % 
% % % %     xLi = xLi - xLi(1);
% % % %     xFi = xFi - xFi(1);
% % % % 
% % % %     % velocity-like signals
% % % %     vL = [0; diff(xLi)] / (dt_ms / 1000);
% % % %     vF = [0; diff(xFi)] / (dt_ms / 1000);
% % % % 
% % % %     % Use region where follower is actually moving
% % % %     moving = abs(vF) > 5;          % mm/s threshold
% % % %     moving = conv(double(moving), ones(5,1), 'same') > 0;
% % % % 
% % % %     if nnz(moving) < 5
% % % %         lag_ms = NaN;
% % % %         return;
% % % %     end
% % % % 
% % % %     vL = vL(moving);
% % % %     vF = vF(moving);
% % % % 
% % % %     vL = vL - mean(vL);
% % % %     vF = vF - mean(vF);
% % % % 
% % % %     [c, lags] = xcorr(vF, vL, 'coeff');
% % % %     [~, idx] = max(c);
% % % %     lag_ms = lags(idx) * dt_ms;
% % % % end
% % % % 
% % % % function tPct = time_to_percent(t, x, pct)
% % % %     xrel = x - x(1);
% % % %     xFinal = xrel(end);
% % % % 
% % % %     if abs(xFinal) < 1e-9
% % % %         tPct = NaN;
% % % %         return;
% % % %     end
% % % % 
% % % %     target = pct * xFinal;
% % % % 
% % % %     if xFinal >= 0
% % % %         idx = find(xrel >= target, 1, 'first');
% % % %     else
% % % %         idx = find(xrel <= target, 1, 'first');
% % % %     end
% % % % 
% % % %     if isempty(idx)
% % % %         tPct = NaN;
% % % %     else
% % % %         tPct = t(idx);
% % % %     end
% % % % end