function plot_poles(ax, poles, marker, color, name)
%PLOT_POLES Mark eigenvalues in the s-plane (with axes through the origin).

hold(ax, 'on');
plot(ax, real(poles), imag(poles), marker, 'Color', color, 'MarkerSize', 9, ...
    'LineWidth', 1.8, 'DisplayName', name);
xline(ax, 0, 'LineStyle', ':', 'HandleVisibility', 'off');
yline(ax, 0, 'LineStyle', ':', 'HandleVisibility', 'off');
grid(ax, 'on');
xlabel(ax, 'Re(s) [1/s]');
ylabel(ax, 'Im(s) [rad/s]');
end
