% Use exactly this style:
%
% x50_y0
% x100_y0
% x100_y20
% x100_y-20

clear; clc; close all;

file_bump = 'BUMP_STRAIGHT_TESTS.xlsx';        % bump sensor data
file_line = 'XY_OFFSET_line.xlsx';             % line sensor data

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
% Compare RMS radial error vs x0
% sqrt(err_x^2 + err_y^2)
% varying lines for y0
% solid = bump, dashed = line
% -----------------------------------
figure('Name','Compare RMS radial error vs x0'); hold on; grid on;

all_y0 = unique([bump.y0, line.y0]);

for k = 1:length(all_y0)
    y0_sel = all_y0(k);

    idxB = find(bump.y0 == y0_sel);
    if ~isempty(idxB)
        [x_sorted, ord] = sort(bump.x0(idxB));
        rms_sorted = bump.rms_r(idxB);
        rms_sorted = rms_sorted(ord);

        plot(x_sorted, rms_sorted, '-o', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Bump, y0 = %g mm', y0_sel));
    end

    idxL = find(line.y0 == y0_sel);
    if ~isempty(idxL)
        [x_sorted, ord] = sort(line.x0(idxL));
        rms_sorted = line.rms_r(idxL);
        rms_sorted = rms_sorted(ord);

        plot(x_sorted, rms_sorted, '--s', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Line, y0 = %g mm', y0_sel));
    end
end

xlabel('Commanded longitudinal separation x0 (mm)');
ylabel('RMS radial error (mm)');
title('Comparison: RMS radial error vs x0');
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
% Compare Final radial error vs x0
% sqrt(final_err_x^2 + final_err_y^2)
% varying lines for y0
% solid = bump, dashed = line
% -----------------------------------
figure('Name','Compare Final radial error vs x0'); hold on; grid on;

all_y0 = unique([bump.y0, line.y0]);

for k = 1:length(all_y0)
    y0_sel = all_y0(k);

    idxB = find(bump.y0 == y0_sel);
    if ~isempty(idxB)
        [x_sorted, ord] = sort(bump.x0(idxB));
        final_sorted = bump.final_r(idxB);
        final_sorted = final_sorted(ord);

        plot(x_sorted, final_sorted, '-o', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Bump, y0 = %g mm', y0_sel));
    end

    idxL = find(line.y0 == y0_sel);
    if ~isempty(idxL)
        [x_sorted, ord] = sort(line.x0(idxL));
        final_sorted = line.final_r(idxL);
        final_sorted = final_sorted(ord);

        plot(x_sorted, final_sorted, '--s', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Line, y0 = %g mm', y0_sel));
    end
end

xlabel('Commanded longitudinal separation x0 (mm)');
ylabel('Final radial error (mm)');
title('Comparison: Final radial error vs x0');
legend show;


% -----------------------------------
% Optional aggregate comparison
% absolute error averaged across all sheets
% -----------------------------------
fprintf('\n=== Aggregate comparison ===\n');

fprintf('Bump mean RMS longitudinal error: %.3f mm\n', mean(abs(bump.rms_x)));
fprintf('Line mean RMS longitudinal error: %.3f mm\n', mean(abs(line.rms_x)));

fprintf('Bump mean RMS lateral error: %.3f mm\n', mean(abs(bump.rms_y)));
fprintf('Line mean RMS lateral error: %.3f mm\n', mean(abs(line.rms_y)));

fprintf('Bump mean RMS radial error: %.3f mm\n', mean(abs(bump.rms_r)));
fprintf('Line mean RMS radial error: %.3f mm\n', mean(abs(line.rms_r)));

fprintf('Bump mean |final longitudinal error|: %.3f mm\n', mean(abs(bump.final_x)));
fprintf('Line mean |final longitudinal error|: %.3f mm\n', mean(abs(line.final_x)));

fprintf('Bump mean |final lateral error|: %.3f mm\n', mean(abs(bump.final_y)));
fprintf('Line mean |final lateral error|: %.3f mm\n', mean(abs(line.final_y)));

fprintf('Bump mean final radial error: %.3f mm\n', mean(abs(bump.final_r)));
fprintf('Line mean final radial error: %.3f mm\n', mean(abs(line.final_r)));

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

if mean(abs(bump.rms_r)) < mean(abs(line.rms_r))
    fprintf('Bump sensors appear better in radial RMS.\n');
else
    fprintf('Line sensors appear better in radial RMS.\n');
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

if mean(abs(bump.final_r)) < mean(abs(line.final_r))
    fprintf('Bump sensors appear better in final radial error.\n');
else
    fprintf('Line sensors appear better in final radial error.\n');
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
    out.rms_r = [];
    out.final_x = [];
    out.final_y = [];
    out.final_r = [];

    figure('Name',['Error X vs Time - ' label]); hold on; grid on;
    xlabel('Time relative to end (ms)');
    ylabel('Longitudinal error (mm)');
    title(['Longitudinal error vs time - ' label]);

    figure('Name',['Error Y vs Time - ' label]); hold on; grid on;
    xlabel('Time relative to end (ms)');
    ylabel('Lateral error (mm)');
    title(['Lateral error vs time - ' label]);

    figure('Name',['Radial Error vs Time - ' label]); hold on; grid on;
    xlabel('Time relative to end (ms)');
    ylabel('Radial error (mm)');
    title(['Radial error vs time - ' label]);

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
        % Error = deviation from commanded/initial separation
        % -----------------------------------
        err_x = sep_x - x0;
        err_y = sep_y - y0;
        err_r = sqrt(err_x.^2 + err_y.^2);

        % RMS error
        rms_x = sqrt(mean(err_x.^2));
        rms_y = sqrt(mean(err_y.^2));
        rms_r = sqrt(mean(err_x.^2 + err_y.^2));

        % Final error
        final_err_x = err_x(end);
        final_err_y = err_y(end);
        final_err_r = sqrt(final_err_x.^2 + final_err_y.^2);

        % Store results
        out.x0(end+1) = x0;
        out.y0(end+1) = y0;
        out.rms_x(end+1) = rms_x;
        out.rms_y(end+1) = rms_y;
        out.rms_r(end+1) = rms_r;
        out.final_x(end+1) = final_err_x;
        out.final_y(end+1) = final_err_y;
        out.final_r(end+1) = final_err_r;

        % Time-domain plots
        figure(findobj('Name',['Error X vs Time - ' label]));
        plot(t_common, err_x, 'DisplayName', sheet);

        figure(findobj('Name',['Error Y vs Time - ' label]));
        plot(t_common, err_y, 'DisplayName', sheet);

        figure(findobj('Name',['Radial Error vs Time - ' label]));
        plot(t_common, err_r, 'DisplayName', sheet);
    end

    figure(findobj('Name',['Error X vs Time - ' label]));
    yline(0,'--k');
    legend show;

    figure(findobj('Name',['Error Y vs Time - ' label]));
    yline(0,'--k');
    legend show;

    figure(findobj('Name',['Radial Error vs Time - ' label]));
    legend show;
end

% % Use exactly this style:
% %
% % x50_y0
% % x100_y0
% % x100_y20
% % x100_y-20
% 
% clear; clc; close all;
% 
% file_bump = 'BUMP_STRAIGHT_TESTS.xlsx';        % bump sensor data
% file_line = 'XY_OFFSET_line.xlsx';    % line sensor data
% 
% % Process both datasets
% bump = process_dataset(file_bump, 'Bump');
% line = process_dataset(file_line, 'Line');
% 
% % -----------------------------------
% % Compare RMS err_x vs x0
% % varying lines for y0
% % solid = bump, dashed = line
% % -----------------------------------
% figure('Name','Compare RMS err_x vs x0'); hold on; grid on;
% 
% all_y0 = unique([bump.y0, line.y0]);
% 
% for k = 1:length(all_y0)
%     y0_sel = all_y0(k);
% 
%     idxB = find(bump.y0 == y0_sel);
%     if ~isempty(idxB)
%         [x_sorted, ord] = sort(bump.x0(idxB));
%         y_sorted = bump.rms_x(idxB);
%         y_sorted = y_sorted(ord);
% 
%         plot(x_sorted, y_sorted, '-o', 'LineWidth', 1.5, ...
%             'DisplayName', sprintf('Bump, y0 = %g mm', y0_sel));
%     end
% 
%     idxL = find(line.y0 == y0_sel);
%     if ~isempty(idxL)
%         [x_sorted, ord] = sort(line.x0(idxL));
%         y_sorted = line.rms_x(idxL);
%         y_sorted = y_sorted(ord);
% 
%         plot(x_sorted, y_sorted, '--s', 'LineWidth', 1.5, ...
%             'DisplayName', sprintf('Line, y0 = %g mm', y0_sel));
%     end
% end
% 
% xlabel('Commanded longitudinal separation x0 (mm)');
% ylabel('RMS longitudinal error (mm)');
% title('Comparison: RMS err_x vs x0');
% legend show;
% 
% 
% % -----------------------------------
% % Compare RMS err_y vs y0
% % varying lines for x0
% % solid = bump, dashed = line
% % -----------------------------------
% figure('Name','Compare RMS err_y vs y0'); hold on; grid on;
% 
% all_x0 = unique([bump.x0, line.x0]);
% 
% for k = 1:length(all_x0)
%     x0_sel = all_x0(k);
% 
%     idxB = find(bump.x0 == x0_sel);
%     if ~isempty(idxB)
%         [y_sorted, ord] = sort(bump.y0(idxB));
%         rms_sorted = bump.rms_y(idxB);
%         rms_sorted = rms_sorted(ord);
% 
%         plot(y_sorted, rms_sorted, '-o', 'LineWidth', 1.5, ...
%             'DisplayName', sprintf('Bump, x0 = %g mm', x0_sel));
%     end
% 
%     idxL = find(line.x0 == x0_sel);
%     if ~isempty(idxL)
%         [y_sorted, ord] = sort(line.y0(idxL));
%         rms_sorted = line.rms_y(idxL);
%         rms_sorted = rms_sorted(ord);
% 
%         plot(y_sorted, rms_sorted, '--s', 'LineWidth', 1.5, ...
%             'DisplayName', sprintf('Line, x0 = %g mm', x0_sel));
%     end
% end
% 
% xlabel('Commanded lateral separation y0 (mm)');
% ylabel('RMS lateral error (mm)');
% title('Comparison: RMS err_y vs y0');
% legend show;
% 
% 
% % -----------------------------------
% % Compare Final err_x vs x0
% % varying lines for y0
% % solid = bump, dashed = line
% % -----------------------------------
% figure('Name','Compare Final err_x vs x0'); hold on; grid on;
% 
% all_y0 = unique([bump.y0, line.y0]);
% 
% for k = 1:length(all_y0)
%     y0_sel = all_y0(k);
% 
%     idxB = find(bump.y0 == y0_sel);
%     if ~isempty(idxB)
%         [x_sorted, ord] = sort(bump.x0(idxB));
%         final_sorted = bump.final_x(idxB);
%         final_sorted = final_sorted(ord);
% 
%         plot(x_sorted, final_sorted, '-o', 'LineWidth', 1.5, ...
%             'DisplayName', sprintf('Bump, y0 = %g mm', y0_sel));
%     end
% 
%     idxL = find(line.y0 == y0_sel);
%     if ~isempty(idxL)
%         [x_sorted, ord] = sort(line.x0(idxL));
%         final_sorted = line.final_x(idxL);
%         final_sorted = final_sorted(ord);
% 
%         plot(x_sorted, final_sorted, '--s', 'LineWidth', 1.5, ...
%             'DisplayName', sprintf('Line, y0 = %g mm', y0_sel));
%     end
% end
% 
% xlabel('Commanded longitudinal separation x0 (mm)');
% ylabel('Final longitudinal error (mm)');
% title('Comparison: Final err_x vs x0');
% yline(0,'--k');
% legend show;
% 
% 
% % -----------------------------------
% % Compare Final err_y vs y0
% % varying lines for x0
% % solid = bump, dashed = line
% % -----------------------------------
% figure('Name','Compare Final err_y vs y0'); hold on; grid on;
% 
% all_x0 = unique([bump.x0, line.x0]);
% 
% for k = 1:length(all_x0)
%     x0_sel = all_x0(k);
% 
%     idxB = find(bump.x0 == x0_sel);
%     if ~isempty(idxB)
%         [y_sorted, ord] = sort(bump.y0(idxB));
%         final_sorted = bump.final_y(idxB);
%         final_sorted = final_sorted(ord);
% 
%         plot(y_sorted, final_sorted, '-o', 'LineWidth', 1.5, ...
%             'DisplayName', sprintf('Bump, x0 = %g mm', x0_sel));
%     end
% 
%     idxL = find(line.x0 == x0_sel);
%     if ~isempty(idxL)
%         [y_sorted, ord] = sort(line.y0(idxL));
%         final_sorted = line.final_y(idxL);
%         final_sorted = final_sorted(ord);
% 
%         plot(y_sorted, final_sorted, '--s', 'LineWidth', 1.5, ...
%             'DisplayName', sprintf('Line, x0 = %g mm', x0_sel));
%     end
% end
% 
% xlabel('Commanded lateral separation y0 (mm)');
% ylabel('Final lateral error (mm)');
% title('Comparison: Final err_y vs y0');
% yline(0,'--k');
% legend show;
% 
% 
% % -----------------------------------
% % Optional aggregate comparison
% % absolute error averaged across all sheets
% % -----------------------------------
% fprintf('\n=== Aggregate comparison ===\n');
% 
% fprintf('Bump mean RMS longitudinal error: %.3f mm\n', mean(abs(bump.rms_x)));
% fprintf('Line mean RMS longitudinal error: %.3f mm\n', mean(abs(line.rms_x)));
% 
% fprintf('Bump mean RMS lateral error: %.3f mm\n', mean(abs(bump.rms_y)));
% fprintf('Line mean RMS lateral error: %.3f mm\n', mean(abs(line.rms_y)));
% 
% fprintf('Bump mean |final longitudinal error|: %.3f mm\n', mean(abs(bump.final_x)));
% fprintf('Line mean |final longitudinal error|: %.3f mm\n', mean(abs(line.final_x)));
% 
% fprintf('Bump mean |final lateral error|: %.3f mm\n', mean(abs(bump.final_y)));
% fprintf('Line mean |final lateral error|: %.3f mm\n', mean(abs(line.final_y)));
% 
% if mean(abs(line.rms_y)) < mean(abs(bump.rms_y))
%     fprintf('Line sensors appear better laterally in RMS.\n');
% else
%     fprintf('Bump sensors appear better laterally in RMS.\n');
% end
% 
% if mean(abs(bump.rms_x)) < mean(abs(line.rms_x))
%     fprintf('Bump sensors appear better longitudinally in RMS.\n');
% else
%     fprintf('Line sensors appear better longitudinally in RMS.\n');
% end
% 
% if mean(abs(line.final_y)) < mean(abs(bump.final_y))
%     fprintf('Line sensors appear better laterally in final error.\n');
% else
%     fprintf('Bump sensors appear better laterally in final error.\n');
% end
% 
% if mean(abs(bump.final_x)) < mean(abs(line.final_x))
%     fprintf('Bump sensors appear better longitudinally in final error.\n');
% else
%     fprintf('Line sensors appear better longitudinally in final error.\n');
% end
% 
% 
% % ===================================
% % Local function
% % ===================================
% function out = process_dataset(filename, label)
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
% 
%     figure('Name',['Error X vs Time - ' label]); hold on; grid on;
%     xlabel('Time relative to end (ms)');
%     ylabel('Longitudinal error (mm)');
%     title(['Longitudinal error vs time - ' label]);
% 
%     figure('Name',['Error Y vs Time - ' label]); hold on; grid on;
%     xlabel('Time relative to end (ms)');
%     ylabel('Lateral error (mm)');
%     title(['Lateral error vs time - ' label]);
% 
%     for s = 1:length(sheetNames)
%         sheet = sheetNames{s};
% 
%         % Parse sheet name like x50_y-20
%         tok = regexp(sheet, 'x([-+]?\d+)_y([-+]?\d+)', 'tokens', 'once');
%         if isempty(tok)
%             fprintf('Skipping sheet "%s" in %s (name not in x##_y## format)\n', sheet, filename);
%             continue;
%         end
% 
%         x0 = str2double(tok{1});
%         y0 = str2double(tok{2});
% 
%         data = readmatrix(filename, 'Sheet', sheet);
% 
%         if size(data,2) < 9
%             fprintf('Skipping sheet "%s" in %s (not enough columns)\n', sheet, filename);
%             continue;
%         end
% 
%         % Leader B:D
%         xL = data(:,2);
%         yL = data(:,3);
%         tL = data(:,4);
% 
%         % Follower G:I
%         xF = data(:,7);
%         yF = data(:,8);
%         tF = data(:,9);
% 
%         % Remove NaNs
%         validL = ~isnan(xL) & ~isnan(yL) & ~isnan(tL);
%         validF = ~isnan(xF) & ~isnan(yF) & ~isnan(tF);
% 
%         xL = xL(validL); yL = yL(validL); tL = tL(validL);
%         xF = xF(validF); yF = yF(validF); tF = tF(validF);
% 
%         if numel(xL) < 5 || numel(xF) < 5
%             fprintf('Skipping sheet "%s" in %s (too few points)\n', sheet, filename);
%             continue;
%         end
% 
%         % -----------------------------------
%         % Align by end of motion
%         % -----------------------------------
%         dxL = [0; diff(xL)];
%         dxF = [0; diff(xF)];
% 
%         moveThresh = 0.5;  % mm per sample
%         idxL_end = find(abs(dxL) > moveThresh, 1, 'last');
%         idxF_end = find(abs(dxF) > moveThresh, 1, 'last');
% 
%         if isempty(idxL_end), idxL_end = length(tL); end
%         if isempty(idxF_end), idxF_end = length(tF); end
% 
%         tL_rel = tL - tL(idxL_end);
%         tF_rel = tF - tF(idxF_end);
% 
%         t_start = max(tL_rel(1), tF_rel(1));
%         t_end   = min(tL_rel(end), tF_rel(end));
% 
%         if t_start >= t_end
%             fprintf('Skipping sheet "%s" in %s (no overlap after alignment)\n', sheet, filename);
%             continue;
%         end
% 
%         t_common = t_start:50:t_end;
% 
%         xL_i = interp1(tL_rel, xL, t_common, 'linear');
%         yL_i = interp1(tL_rel, yL, t_common, 'linear');
% 
%         xF_i = interp1(tF_rel, xF, t_common, 'linear');
%         yF_i = interp1(tF_rel, yF, t_common, 'linear');
% 
%         % -----------------------------------
%         % Put leader into follower coordinate frame
%         % Leader reverses, so flip sign first
%         % -----------------------------------
%         xL_F = -xL_i + x0;
%         yL_F = -yL_i + y0;
% 
%         % -----------------------------------
%         % Actual separation
%         % -----------------------------------
%         sep_x = xL_F - xF_i;
%         sep_y = yL_F - yF_i;
% 
%         % -----------------------------------
%         % Error = deviation from commanded/initial separation
%         % -----------------------------------
%         err_x = sep_x - x0;
%         err_y = sep_y - y0;
% 
%         % RMS error
%         rms_x = sqrt(mean(err_x.^2));
%         rms_y = sqrt(mean(err_y.^2));
% 
%         % Final error
%         final_err_x = err_x(end);
%         final_err_y = err_y(end);
% 
%         % Store results
%         out.x0(end+1) = x0;
%         out.y0(end+1) = y0;
%         out.rms_x(end+1) = rms_x;
%         out.rms_y(end+1) = rms_y;
%         out.final_x(end+1) = final_err_x;
%         out.final_y(end+1) = final_err_y;
% 
%         % Time-domain plots
%         figure(findobj('Name',['Error X vs Time - ' label]));
%         plot(t_common, err_x, 'DisplayName', sheet);
% 
%         figure(findobj('Name',['Error Y vs Time - ' label]));
%         plot(t_common, err_y, 'DisplayName', sheet);
%     end
% 
%     figure(findobj('Name',['Error X vs Time - ' label]));
%     yline(0,'--k');
%     legend show;
% 
%     figure(findobj('Name',['Error Y vs Time - ' label]));
%     yline(0,'--k');
%     legend show;
% end