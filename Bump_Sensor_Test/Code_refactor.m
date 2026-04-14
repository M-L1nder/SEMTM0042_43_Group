% Use exactly this style:
%
% x50_y0
% x100_y0
% x100_y20
% x100_y-20

clear; clc; close all;

filename = 'XY_offset2.xlsx';   % <-- change this

[~, sheetNames] = xlsfinfo(filename);

all_x0 = [];
all_y0 = [];

all_rms_x = [];
all_rms_y = [];

all_final_x = [];
all_final_y = [];

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

    x0 = str2double(tok{1});   % commanded longitudinal separation in mm
    y0 = str2double(tok{2});   % commanded lateral separation in mm

    data = readmatrix(filename, 'Sheet', sheet);

    if size(data,2) < 9
        fprintf('Skipping sheet "%s" (not enough columns)\n', sheet);
        continue;
    end

    % Leader B:D
    xL = data(:,2);
    yL = data(:,3);
    tL = data(:,4);

    % Follower G:I
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
    % Put leader into follower coordinate frame
    % Leader reverses, so flip sign first
    % -----------------------------------
    xL_F = -xL_i + x0;
    yL_F = -yL_i + y0;

    % -----------------------------------
    % Actual separation
    % -----------------------------------
    sep_x = xL_F - xF_i;
    sep_y = yL_F - yF_i;

    % -----------------------------------
    % Error = deviation from commanded/initial separation
    % -----------------------------------
    err_x = sep_x - x0;
    err_y = sep_y - y0;

    % RMS error
    rms_x = sqrt(mean(err_x.^2));
    rms_y = sqrt(mean(err_y.^2));

    % Final error
    final_err_x = err_x(end);
    final_err_y = err_y(end);

    % Store results
    all_x0(end+1) = x0;
    all_y0(end+1) = y0;
    all_rms_x(end+1) = rms_x;
    all_rms_y(end+1) = rms_y;
    all_final_x(end+1) = final_err_x;
    all_final_y(end+1) = final_err_y;

    % Time-domain plots
    figure(findobj('Name','Error X vs Time'));
    plot(t_common, err_x, 'DisplayName', sheet);

    figure(findobj('Name','Error Y vs Time'));
    plot(t_common, err_y, 'DisplayName', sheet);

    % Optional debug plots
    % figure('Name',['Debug_' sheet]);
    %
    % subplot(4,1,1);
    % plot(t_common, xL_F, 'LineWidth', 1.2); hold on;
    % plot(t_common, xF_i, 'LineWidth', 1.2);
    % grid on;
    % ylabel('x in follower frame (mm)');
    % legend('Leader in follower frame','Follower');
    % title(['Common-frame x traces: ' sheet]);
    %
    % subplot(4,1,2);
    % plot(t_common, yL_F, 'LineWidth', 1.2); hold on;
    % plot(t_common, yF_i, 'LineWidth', 1.2);
    % grid on;
    % ylabel('y in follower frame (mm)');
    % legend('Leader in follower frame','Follower');
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

% -----------------------------------
% RMS err_x against x0, varying lines for y0
% -----------------------------------
figure('Name','RMS err_x vs x0'); hold on; grid on;
unique_y0 = unique(all_y0);

for k = 1:length(unique_y0)
    y0_sel = unique_y0(k);
    idx = find(all_y0 == y0_sel);

    [x_sorted, ord] = sort(all_x0(idx));
    rms_sorted = all_rms_x(idx);
    rms_sorted = rms_sorted(ord);

    plot(x_sorted, rms_sorted, '-o', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('y0 = %g mm', y0_sel));
end

xlabel('Commanded longitudinal separation x0 (mm)');
ylabel('RMS longitudinal error (mm)');
title('RMS err_x vs x0');
yline(0,'--k');
legend show;

% -----------------------------------
% RMS err_y against y0, varying lines for x0
% -----------------------------------
figure('Name','RMS err_y vs y0'); hold on; grid on;
unique_x0 = unique(all_x0);

for k = 1:length(unique_x0)
    x0_sel = unique_x0(k);
    idx = find(all_x0 == x0_sel);

    [y_sorted, ord] = sort(all_y0(idx));
    rms_sorted = all_rms_y(idx);
    rms_sorted = rms_sorted(ord);

    plot(y_sorted, rms_sorted, '-o', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('x0 = %g mm', x0_sel));
end

xlabel('Commanded lateral separation y0 (mm)');
ylabel('RMS lateral error (mm)');
title('RMS err_y vs y0');
yline(0,'--k');
legend show;

% -----------------------------------
% Final err_x against x0, varying lines for y0
% -----------------------------------
figure('Name','Final err_x vs x0'); hold on; grid on;
unique_y0 = unique(all_y0);

for k = 1:length(unique_y0)
    y0_sel = unique_y0(k);
    idx = find(all_y0 == y0_sel);

    [x_sorted, ord] = sort(all_x0(idx));
    final_sorted = all_final_x(idx);
    final_sorted = final_sorted(ord);

    plot(x_sorted, final_sorted, '-o', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('y0 = %g mm', y0_sel));
end

xlabel('Commanded longitudinal separation x0 (mm)');
ylabel('Final longitudinal error (mm)');
title('Final err_x vs x0');
yline(0,'--k');
legend show;

% -----------------------------------
% Final err_y against y0, varying lines for x0
% -----------------------------------
figure('Name','Final err_y vs y0'); hold on; grid on;
unique_x0 = unique(all_x0);

for k = 1:length(unique_x0)
    x0_sel = unique_x0(k);
    idx = find(all_x0 == x0_sel);

    [y_sorted, ord] = sort(all_y0(idx));
    final_sorted = all_final_y(idx);
    final_sorted = final_sorted(ord);

    plot(y_sorted, final_sorted, '-o', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('x0 = %g mm', x0_sel));
end

xlabel('Commanded lateral separation y0 (mm)');
ylabel('Final lateral error (mm)');
title('Final err_y vs y0');
yline(0,'--k');
legend show;