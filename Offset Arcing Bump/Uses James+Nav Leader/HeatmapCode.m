clear; clc; close all;

filename = 'heatmap_data.xlsx';

%% -------------------- LOAD LINE DATA --------------------
Tline = readtable(filename, 'Sheet', 'Line');

xL = Tline.x_mm;
yL = Tline.y_mm;

if ismember('filteredTotal', Tline.Properties.VariableNames)
    zL = Tline.filteredTotal;
else
    zL = Tline.total;
end

validL = ~(isnan(xL) | isnan(yL) | isnan(zL));
xL = xL(validL);
yL = yL(validL);
zL = zL(validL);

XYL = [xL yL];
[uniqueXYL, ~, icL] = unique(XYL, 'rows');
zL_mean = accumarray(icL, zL, [], @mean);
xL_mean = uniqueXYL(:,1);
yL_mean = uniqueXYL(:,2);

%% -------------------- LOAD BUMP DATA --------------------
Tbump = readtable(filename, 'Sheet', 'bump');

xB = Tbump.x_mm;
yB = Tbump.y_mm;
zB = Tbump.total;

validB = ~(isnan(xB) | isnan(yB) | isnan(zB));
xB = xB(validB);
yB = yB(validB);
zB = zB(validB);

XYB = [xB yB];
[uniqueXYB, ~, icB] = unique(XYB, 'rows');
zB_mean = accumarray(icB, zB, [], @mean);
xB_mean = uniqueXYB(:,1);
yB_mean = uniqueXYB(:,2);

%% -------------------- COMMON GRID --------------------
xmin = min([xL_mean; xB_mean]);
xmax = max([xL_mean; xB_mean]);
ymin = min([yL_mean; yB_mean]);
ymax = max([yL_mean; yB_mean]);

xq = linspace(xmin, xmax, 150);
yq = linspace(ymin, ymax, 150);
[Xq, Yq] = meshgrid(xq, yq);

ZL = griddata(xL_mean, yL_mean, zL_mean, Xq, Yq, 'natural');
ZB = griddata(xB_mean, yB_mean, zB_mean, Xq, Yq, 'natural');

%% -------------------- SHARED COLOUR SCALE --------------------
allMin = min([ZL(:); ZB(:)], [], 'omitnan');
allMax = max([ZL(:); ZB(:)], [], 'omitnan');

%% -------------------- FIGURE SETUP --------------------
fig = figure('Color', 'w', ...
    'Units', 'centimeters', ...
    'Position', [2 2 15 6]);

t = tiledlayout(fig, 1, 2, ...
    'Padding', 'compact', ...
    'TileSpacing', 'compact');

nLevels = 20;

%% -------------------- LINE CONTOUR --------------------
ax1 = nexttile;
contourf(ax1, Xq, Yq, ZL, nLevels, 'LineColor', 'none');
hold(ax1, 'on');
plot(ax1, xL_mean, yL_mean, 'k.', 'MarkerSize', 6);
hold(ax1, 'off');
rectangle(ax1, ...
    'Position', [100, -20, 20, 40], ...
    'EdgeColor', 'k', ...
    'LineWidth', 1.5, ...
    'LineStyle', '--');
% axis(ax1, 'equal');
axis(ax1, 'tight');
xlim(ax1, [xmin xmax]);
ylim(ax1, [ymin ymax]);

xlabel(ax1, ' X (mm)', 'FontSize', 12);
ylabel(ax1, ' Y (mm)', 'FontSize', 12);
% title(ax1, 'Line sensors', 'FontSize', 12);

set(ax1, ...
    'FontSize', 12, ...
    'LineWidth', 1, ...
    'Box', 'on', ...
    'Layer', 'top');

clim(ax1, [allMin allMax]);

%% -------------------- BUMP CONTOUR --------------------
ax2 = nexttile;
contourf(ax2, Xq, Yq, ZB, nLevels, 'LineColor', 'none');
hold(ax2, 'on');
plot(ax2, xB_mean, yB_mean, 'k.', 'MarkerSize', 6);
hold(ax2, 'off');
rectangle(ax2, ...
    'Position', [100, -20, 20, 40], ...
    'EdgeColor', 'k', ...
    'LineWidth', 1.5, ...
    'LineStyle', '--');
% axis(ax2, 'equal');
axis(ax2, 'tight');
xlim(ax2, [xmin xmax]);
ylim(ax2, [ymin ymax]);

xlabel(ax2, ' X (mm)', 'FontSize', 12);
ylabel(ax2, '', 'FontSize', 12);
% title(ax2, 'Bump sensors', 'FontSize', 12);

set(ax2, ...
    'FontSize', 12, ...
    'LineWidth', 1, ...
    'Box', 'on', ...
    'Layer', 'top');

clim(ax2, [allMin allMax]);

%% -------------------- ONE SHARED COLOURBAR --------------------
cb = colorbar(ax2, 'eastoutside');
cb.Layout.Tile = 'east';
cb.FontSize = 12;
cb.LineWidth = 1;
cb.Label.String = sprintf('Normalized signal (%%)');
cb.Label.FontSize = 12;

%% -------------------- COLORMAP --------------------
colormap(fig, parula);