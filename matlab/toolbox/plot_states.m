function plot_states(ax, t, X, labels, line_style)
%PLOT_STATES Plot the columns of X against t using the project colours.
%   PLOT_STATES(ax, t, X, labels) draws one line per column with legend entries from
%   LABELS (string array or cell array, TeX markup allowed).
%   PLOT_STATES(ax, t, X, labels, line_style) sets the line style, e.g. '--'.

if nargin < 5
    line_style = '-';
end
colors = studio_colors();
hold(ax, 'on');
for k = 1:size(X, 2)
    plot(ax, t, X(:, k), line_style, 'Color', colors(mod(k - 1, size(colors, 1)) + 1, :), ...
        'LineWidth', 1.6, 'DisplayName', char(labels(k)));
end
grid(ax, 'on');
box(ax, 'off');
end
