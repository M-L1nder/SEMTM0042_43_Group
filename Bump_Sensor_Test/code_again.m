% Use exactly this style:
%
% x50_y0
% x100_y0
% x100_y20
% x100_y-20

clear; clc; close all;

file_bump = 'XY_offset2.xlsx';        % bump sensor data
file_line = 'XY_OFFSET_line.xlsx';    % line sensor data

% Process both datasets
bump = process_dataset(file_bump, 'Bump');
line = process_dataset(file_line, 'Line');

% -----------------------------------
% Compare RMS err_x vs x0
% varying lines for y0
% solid = bump, dashed = line
% -----------------------------------
figure('Name','Compare RMS err_x vs x0'); hold on; grid on;

all_y0 = unique([bump.y0, line.y0]);

for k = 1:length(all_y0)
    y0_sel = all_y0(k);

    idxB = find(bump.y0 == y0_sel);
    if ~isempty(idxB)
        [x_sorted, ord] = sort(bump.x0(idxB));
        y_sorted = bump.rms_x(idxB);
        y_sorted = y_sorted(ord);

        plot(x_sorted, y_sorted, '-o', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Bump, y0 = %g mm', y0_sel));
    end

    idxL = find(line.y0 == y0_sel);
    if ~isempty(idxL)
        [x_sorted, ord] = sort(line.x0(idxL));
        y_sorted = line.rms_x(idxL);
        y_sorted = y_sorted(ord);

        plot(x_sorted, y_sorted, '--s', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Line, y0 = %g mm', y0_sel));
    end
end

xlabel('Commanded longitudinal separation x0 (mm)');
ylabel('RMS longitudinal error (mm)');
title('Comparison: RMS err_x vs x0');
legend show;


% -----------------------------------
% Compare RMS err_y vs y0
% varying lines for x0
% solid = bump, dashed = line
% -----------------------------------
figure('Name','Compare RMS err_y vs y0'); hold on; grid on;

all_x0 = unique([bump.x0, line.x0]);

for k = 1:length(all_x0)
    x0_sel = all_x0(k);

    idxB = find(bump.x0 == x0_sel);
    if ~isempty(idxB)
        [y_sorted, ord] = sort(bump.y0(idxB));
        rms_sorted = bump.rms_y(idxB);
        rms_sorted = rms_sorted(ord);

        plot(y_sorted, rms_sorted, '-o', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Bump, x0 = %g mm', x0_sel));
    end

    idxL = find(line.x0 == x0_sel);
    if ~isempty(idxL)
        [y_sorted, ord] = sort(line.y0(idxL));
        rms_sorted = line.rms_y(idxL);
        rms_sorted = rms_sorted(ord);

        plot(y_sorted, rms_sorted, '--s', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Line, x0 = %g mm', x0_sel));
    end
end

xlabel('Commanded lateral separation y0 (mm)');
ylabel('RMS lateral error (mm)');
title('Comparison: RMS err_y vs y0');
legend show;


% -----------------------------------
% Compare Final err_x vs x0
% varying lines for y0
% solid = bump, dashed = line
% -----------------------------------
figure('Name','Compare Final err_x vs x0'); hold on; grid on;

all_y0 = unique([bump.y0, line.y0]);

for k = 1:length(all_y0)
    y0_sel = all_y0(k);

    idxB = find(bump.y0 == y0_sel);
    if ~isempty(idxB)
        [x_sorted, ord] = sort(bump.x0(idxB));
        final_sorted = bump.final_x(idxB);
        final_sorted = final_sorted(ord);

        plot(x_sorted, final_sorted, '-o', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Bump, y0 = %g mm', y0_sel));
    end

    idxL = find(line.y0 == y0_sel);
    if ~isempty(idxL)
        [x_sorted, ord] = sort(line.x0(idxL));
        final_sorted = line.final_x(idxL);
        final_sorted = final_sorted(ord);

        plot(x_sorted, final_sorted, '--s', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Line, y0 = %g mm', y0_sel));
    end
end

xlabel('Commanded longitudinal separation x0 (mm)');
ylabel('Final longitudinal error (mm)');
title('Comparison: Final err_x vs x0');
yline(0,'--k');
legend show;


% -----------------------------------
% Compare Final err_y vs y0
% varying lines for x0
% solid = bump, dashed = line
% -----------------------------------
figure('Name','Compare Final err_y vs y0'); hold on; grid on;

all_x0 = unique([bump.x0, line.x0]);

for k = 1:length(all_x0)
    x0_sel = all_x0(k);

    idxB = find(bump.x0 == x0_sel);
    if ~isempty(idxB)
        [y_sorted, ord] = sort(bump.y0(idxB));
        final_sorted = bump.final_y(idxB);
        final_sorted = final_sorted(ord);

        plot(y_sorted, final_sorted, '-o', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Bump, x0 = %g mm', x0_sel));
    end

    idxL = find(line.x0 == x0_sel);
    if ~isempty(idxL)
        [y_sorted, ord] = sort(line.y0(idxL));
        final_sorted = line.final_y(idxL);
        final_sorted = final_sorted(ord);

        plot(y_sorted, final_sorted, '--s', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Line, x0 = %g mm', x0_sel));
    end
end

xlabel('Commanded lateral separation y0 (mm)');
ylabel('Final lateral error (mm)');
title('Comparison: Final err_y vs y0');
yline(0,'--k');
legend show;


% -----------------------------------
% Compare RMS displacement error
% desired displacement magnitude = sqrt(x0^2 + y0^2)
% -----------------------------------
figure('Name','Compare RMS displacement error'); hold on; grid on;

plot(bump.sep_mag_des, bump.rms_mag, 'o', 'LineWidth', 1.5, ...
    'DisplayName', 'Bump');
plot(line.sep_mag_des, line.rms_mag, 's', 'LineWidth', 1.5, ...
    'DisplayName', 'Line');

xlabel('Desired separation magnitude (mm)');
ylabel('RMS displacement error (mm)');
title('Comparison: RMS displacement error');
legend show;


% -----------------------------------
% Compare Final displacement error
% -----------------------------------
figure('Name','Compare Final displacement error'); hold on; grid on;

plot(bump.sep_mag_des, bump.final_mag, 'o', 'LineWidth', 1.5, ...
    'DisplayName', 'Bump');
plot(line.sep_mag_des, line.final_mag, 's', 'LineWidth', 1.5, ...
    'DisplayName', 'Line');

xlabel('Desired separation magnitude (mm)');
ylabel('Final displacement error (mm)');
title('Comparison: Final displacement error');
yline(0,'--k');
legend show;


% -----------------------------------
% Compare displacement error vs x0
% varying lines for y0
% solid = bump, dashed = line
% -----------------------------------
figure('Name','Compare RMS displacement error vs x0'); hold on; grid on;

all_y0 = unique([bump.y0, line.y0]);

for k = 1:length(all_y0)
    y0_sel = all_y0(k);

    idxB = find(bump.y0 == y0_sel);
    if ~isempty(idxB)
        [x_sorted, ord] = sort(bump.x0(idxB));
        mag_sorted = bump.rms_mag(idxB);
        mag_sorted = mag_sorted(ord);

        plot(x_sorted, mag_sorted, '-o', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Bump, y0 = %g mm', y0_sel));
    end

    idxL = find(line.y0 == y0_sel);
    if ~isempty(idxL)
        [x_sorted, ord] = sort(line.x0(idxL));
        mag_sorted = line.rms_mag(idxL);
        mag_sorted = mag_sorted(ord);

        plot(x_sorted, mag_sorted, '--s', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Line, y0 = %g mm', y0_sel));
    end
end

xlabel('Commanded longitudinal separation x0 (mm)');
ylabel('RMS displacement error (mm)');
title('Comparison: RMS displacement error vs x0');
legend show;


figure('Name','Compare Final displacement error vs x0'); hold on; grid on;

for k = 1:length(all_y0)
    y0_sel = all_y0(k);

    idxB = find(bump.y0 == y0_sel);
    if ~isempty(idxB)
        [x_sorted, ord] = sort(bump.x0(idxB));
        mag_sorted = bump.final_mag(idxB);
        mag_sorted = mag_sorted(ord);

        plot(x_sorted, mag_sorted, '-o', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Bump, y0 = %g mm', y0_sel));
    end

    idxL = find(line.y0 == y0_sel);
    if ~isempty(idxL)
        [x_sorted, ord] = sort(line.x0(idxL));
        mag_sorted = line.final_mag(idxL);
        mag_sorted = mag_sorted(ord);

        plot(x_sorted, mag_sorted, '--s', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Line, y0 = %g mm', y0_sel));
    end
end

xlabel('Commanded longitudinal separation x0 (mm)');
ylabel('Final displacement error (mm)');
title('Comparison: Final displacement error vs x0');
yline(0,'--k');
legend show;


% -----------------------------------
% Compare displacement error vs y0
% varying lines for x0
% solid = bump, dashed = line
% -----------------------------------
figure('Name','Compare RMS displacement error vs y0'); hold on; grid on;

all_x0 = unique([bump.x0, line.x0]);

for k = 1:length(all_x0)
    x0_sel = all_x0(k);

    idxB = find(bump.x0 == x0_sel);
    if ~isempty(idxB)
        [y_sorted, ord] = sort(bump.y0(idxB));
        mag_sorted = bump.rms_mag(idxB);
        mag_sorted = mag_sorted(ord);

        plot(y_sorted, mag_sorted, '-o', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Bump, x0 = %g mm', x0_sel));
    end

    idxL = find(line.x0 == x0_sel);
    if ~isempty(idxL)
        [y_sorted, ord] = sort(line.y0(idxL));
        mag_sorted = line.rms_mag(idxL);
        mag_sorted = mag_sorted(ord);

        plot(y_sorted, mag_sorted, '--s', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Line, x0 = %g mm', x0_sel));
    end
end

xlabel('Commanded lateral separation y0 (mm)');
ylabel('RMS displacement error (mm)');
title('Comparison: RMS displacement error vs y0');
legend show;


figure('Name','Compare Final displacement error vs y0'); hold on; grid on;

for k = 1:length(all_x0)
    x0_sel = all_x0(k);

    idxB = find(bump.x0 == x0_sel);
    if ~isempty(idxB)
        [y_sorted, ord] = sort(bump.y0(idxB));
        mag_sorted = bump.final_mag(idxB);
        mag_sorted = mag_sorted(ord);

        plot(y_sorted, mag_sorted, '-o', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Bump, x0 = %g mm', x0_sel));
    end

    idxL = find(line.x0 == x0_sel);
    if ~isempty(idxL)
        [y_sorted, ord] = sort(line.y0(idxL));
        mag_sorted = line.final_mag(idxL);
        mag_sorted = mag_sorted(ord);

        plot(y_sorted, mag_sorted, '--s', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Line, x0 = %g mm', x0_sel));
    end
end

xlabel('Commanded lateral separation y0 (mm)');
ylabel('Final displacement error (mm)');
title('Comparison: Final displacement error vs y0');
yline(0,'--k');
legend show;


% -----------------------------------
% Aggregate comparison
% -----------------------------------
fprintf('\n=== Aggregate comparison ===\n');

fprintf('Bump mean RMS longitudinal error: %.3f mm\n', mean(abs(bump.rms_x)));
fprintf('Line mean RMS longitudinal error: %.3f mm\n', mean(abs(line.rms_x)));

fprintf('Bump mean RMS lateral error: %.3f mm\n', mean(abs(bump.rms_y)));
fprintf('Line mean RMS lateral error: %.3f mm\n', mean(abs(line.rms_y)));

fprintf('Bump mean |final longitudinal error|: %.3f mm\n', mean(abs(bump.final_x)));
fprintf('Line mean |final longitudinal error|: %.3f mm\n', mean(abs(line.final_x)));

fprintf('Bump mean |final lateral error|: %.3f mm\n', mean(abs(bump.final_y)));
fprintf('Line mean |final lateral error|: %.3f mm\n', mean(abs(line.final_y)));

fprintf('Bump mean RMS displacement error: %.3f mm\n', mean(abs(bump.rms_mag)));
fprintf('Line mean RMS displacement error: %.3f mm\n', mean(abs(line.rms_mag)));

fprintf('Bump mean |final displacement error|: %.3f mm\n', mean(abs(bump.final_mag)));
fprintf('Line mean |final displacement error|: %.3f mm\n', mean(abs(line.final_mag)));

if mean(abs(line.rms_y)) < mean(abs(bump.rms_y))
    fprintf('Line sensors appear better laterally in RMS.\n');
else
    fprintf('Bump sensors appear better laterally in RMS.\n');
end

if mean(abs(bump.rms_x)) < mean(abs(line.rms_x))
    fprintf('Bump sensors appear better longitudinally in RMS.\n');
else
    fprintf('Line sensors appear better longitudinally in RMS.\n');
end

if mean(abs(line.final_y)) < mean(abs(bump.final_y))
    fprintf('Line sensors appear better laterally in final error.\n');
else
    fprintf('Bump sensors appear better laterally in final error.\n');
end

if mean(abs(bump.final_x)) < mean(abs(line.final_x))
    fprintf('Bump sensors appear better longitudinally in final error.\n');
else
    fprintf('Line sensors appear better longitudinally in final error.\n');
end

if mean(abs(line.rms_mag)) < mean(abs(bump.rms_mag))
    fprintf('Line sensors maintain separation magnitude better in RMS.\n');
else
    fprintf('Bump sensors maintain separation magnitude better in RMS.\n');
end

if mean(abs(line.final_mag)) < mean(abs(bump.final_mag))
    fprintf('Line sensors maintain separation magnitude better in final error.\n');
else
    fprintf('Bump sensors maintain separation magnitude better in final error.\n');
end


% ===================================
% Local function
% ===================================
function out = process_dataset(filename, label)

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

    figure('Name',['Error X vs Time - ' label]); hold on; grid on;
    xlabel('Time relative to end (ms)');
    ylabel('Longitudinal error (mm)');
    title(['Longitudinal error vs time - ' label]);

    figure('Name',['Error Y vs Time - ' label]); hold on; grid on;
    xlabel('Time relative to end (ms)');
    ylabel('Lateral error (mm)');
    title(['Lateral error vs time - ' label]);

    figure('Name',['Displacement Error vs Time - ' label]); hold on; grid on;
    xlabel('Time relative to end (ms)');
    ylabel('Displacement error (mm)');
    title(['Displacement error vs time - ' label]);

    for s = 1:length(sheetNames)
        sheet = sheetNames{s};

        % Parse sheet name like x50_y-20
        tok = regexp(sheet, 'x([-+]?\d+)_y([-+]?\d+)', 'tokens', 'once');
        if isempty(tok)
            fprintf('Skipping sheet "%s" in %s (name not in x##_y## format)\n', sheet, filename);
            continue;
        end

        x0 = str2double(tok{1});
        y0 = str2double(tok{2});

        data = readmatrix(filename, 'Sheet', sheet);

        if size(data,2) < 9
            fprintf('Skipping sheet "%s" in %s (not enough columns)\n', sheet, filename);
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
            fprintf('Skipping sheet "%s" in %s (too few points)\n', sheet, filename);
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
            fprintf('Skipping sheet "%s" in %s (no overlap after alignment)\n', sheet, filename);
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
        % Component-wise error
        % -----------------------------------
        err_x = sep_x - x0;
        err_y = sep_y - y0;

        % -----------------------------------
        % Displacement magnitude error
        % -----------------------------------
        sep_mag = sqrt(sep_x.^2 + sep_y.^2);
        sep_mag_des = sqrt(x0.^2 + y0.^2);
        err_mag = sep_mag - sep_mag_des;

        % RMS metrics
        rms_x = sqrt(mean(err_x.^2));
        rms_y = sqrt(mean(err_y.^2));
        rms_mag = sqrt(mean(err_mag.^2));

        % Final metrics
        final_err_x = err_x(end);
        final_err_y = err_y(end);
        final_err_mag = err_mag(end);

        % Store results
        out.x0(end+1) = x0;
        out.y0(end+1) = y0;
        out.rms_x(end+1) = rms_x;
        out.rms_y(end+1) = rms_y;
        out.final_x(end+1) = final_err_x;
        out.final_y(end+1) = final_err_y;
        out.sep_mag_des(end+1) = sep_mag_des;
        out.rms_mag(end+1) = rms_mag;
        out.final_mag(end+1) = final_err_mag;

        % Time-domain plots
        figure(findobj('Name',['Error X vs Time - ' label]));
        plot(t_common, err_x, 'DisplayName', sheet);

        figure(findobj('Name',['Error Y vs Time - ' label]));
        plot(t_common, err_y, 'DisplayName', sheet);

        figure(findobj('Name',['Displacement Error vs Time - ' label]));
        plot(t_common, err_mag, 'DisplayName', sheet);
    end

    figure(findobj('Name',['Error X vs Time - ' label]));
    yline(0,'--k');
    legend show;

    figure(findobj('Name',['Error Y vs Time - ' label]));
    yline(0,'--k');
    legend show;

    figure(findobj('Name',['Displacement Error vs Time - ' label]));
    yline(0,'--k');
    legend show;
end