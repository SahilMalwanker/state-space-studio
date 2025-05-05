function results = study_01_fundamentals(opts)
%STUDY_01_FUNDAMENTALS Free and forced responses of three introductory plants.
%   results = STUDY_01_FUNDAMENTALS() analyses the RC ladder, the coupled two-input
%   system and the mass-spring-damper: poles, responses to initial conditions, steps
%   and a ramp, 3-D state trajectories and a phase portrait.
%   results = STUDY_01_FUNDAMENTALS(opts) exports figures when opts.figure_dir is set.
%
%   Python counterpart: state_space_studio.scenarios.fundamentals_study

if nargin < 1
    opts = struct();
end
opts = merge_params(struct('figure_dir', ""), opts);
colors = studio_colors();

%% RC ladder: unit step and free response from v1(0) = 0.5 V
rc = rc_ladder_model();
results.rc = structural_analysis(rc);
rc_states = ss(rc.A, rc.B, eye(3), zeros(3, 1));
t = (0:0.25:150)';
x_step = lsim(rc_states, ones(size(t)), t);
x_free = lsim(rc_states, zeros(size(t)), t, [0.5; 0; 0]);

fig = figure('Name', 'RC ladder', 'Color', 'w');
ax = axes(fig);
plot_states(ax, t, x_step, ["v_1, unit step", "v_2, unit step", "v_3, unit step"]);
plot_states(ax, t, x_free, ["v_1, free", "v_2, free", "v_3, free"], '--');
yline(ax, 1, 'LineStyle', ':', 'HandleVisibility', 'off');
title(ax, 'RC ladder: unit step (solid) and free response from v_1(0) = 0.5 V (dashed)');
xlabel(ax, 'time [s]');
ylabel(ax, 'capacitor voltage [V]');
legend(ax, 'Location', 'east');
export_figure(fig, opts, 'rc_ladder');

%% Coupled two-input system
mimo = coupled_mimo_model();
results.mimo = structural_analysis(mimo);
mimo_states = ss(mimo.A, mimo.B, eye(3), zeros(3, 2));
t = (0:0.01:8)';
no_input = zeros(numel(t), 2);
x_a = lsim(mimo_states, no_input, t, [0.2; 0; 0]);
x_b = lsim(mimo_states, no_input, t, [0; 0.1; 0]);
x_step = lsim(mimo_states, repmat([3 1], numel(t), 1), t);
x_ramp = lsim(mimo_states, [t, zeros(size(t))], t);
results.mimo_ramp_final = x_ramp(end, :);

fig = figure('Name', 'Coupled system', 'Color', 'w', 'Position', [100 100 1100 750]);
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(layout);
plot_poles(ax, results.mimo.poles, 'x', colors(4, :), 'poles');
title(ax, 'Poles: -3 and -1 \pm 4j');
ax = nexttile(layout);
plot_states(ax, t, x_a, ["x_1, x_0 = (0.2, 0, 0)", "x_2", "x_3"]);
plot_states(ax, t, x_b, ["x_1, x_0 = (0, 0.1, 0)", "x_2", "x_3"], '--');
title(ax, 'Free response from two initial conditions');
xlabel(ax, 'time [s]');
legend(ax, 'NumColumns', 2);
ax = nexttile(layout);
plot3(ax, x_a(:, 1), x_a(:, 2), x_a(:, 3), 'Color', colors(1, :), 'LineWidth', 1.8);
hold(ax, 'on');
plot3(ax, x_b(:, 1), x_b(:, 2), x_b(:, 3), 'Color', colors(4, :), 'LineWidth', 1.8);
plot3(ax, 0, 0, 0, 'k*', 'MarkerSize', 10);
grid(ax, 'on');
view(ax, 3);
xlabel(ax, 'x_1');
ylabel(ax, 'x_2');
zlabel(ax, 'x_3');
legend(ax, 'x_0 = (0.2, 0, 0)', 'x_0 = (0, 0.1, 0)', 'equilibrium');
title(ax, 'State-space trajectories');
ax = nexttile(layout);
plot_states(ax, t, x_step, ["x_1, u = (3, 1)", "x_2", "x_3"]);
plot_states(ax, t, x_ramp, ["x_1, u = (t, 0)", "x_2", "x_3"], '--');
title(ax, 'Step input (solid) and ramp input (dashed)');
xlabel(ax, 'time [s]');
legend(ax, 'NumColumns', 2, 'Location', 'northwest');
export_figure(fig, opts, 'coupled_system');

%% Mass-spring-damper: step response and phase portrait
msd = mass_spring_damper_model();
results.msd = structural_analysis(msd);
msd_states = ss(msd.A, msd.B, eye(2), zeros(2, 1));
t = (0:0.002:1.5)';
x_step = lsim(msd_states, ones(size(t)), t);
results.msd_static_position = x_step(end, 1);

fig = figure('Name', 'Mass-spring-damper', 'Color', 'w', 'Position', [100 100 1100 420]);
layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(layout);
plot_states(ax, t, x_step, ["position x [m]", "velocity v [m/s]"]);
title(ax, 'Step response to u = 1 N (static deflection u / c = 0.25 m)');
xlabel(ax, 'time [s]');
legend(ax);

ax = nexttile(layout);
[X, Vel] = meshgrid(linspace(-0.36, 0.36, 21), linspace(-3.6, 3.6, 21));
dX = msd.A(1, 1) * X + msd.A(1, 2) * Vel;
dV = msd.A(2, 1) * X + msd.A(2, 2) * Vel;
arrow_length = hypot(dX / 0.72, dV / 7.2);
arrow_length(arrow_length == 0) = 1;
quiver(ax, X, Vel, 0.025 * 0.72 * dX ./ arrow_length, 0.025 * 7.2 * dV ./ arrow_length, 0, ...
    'Color', [0.78 0.80 0.84], 'HandleVisibility', 'off');
hold(ax, 'on');
for angle = (0:11) * pi / 6
    [~, ~, x_free] = initial(msd_states, [0.3 * cos(angle); 3 * sin(angle)], t);
    plot(ax, x_free(:, 1), x_free(:, 2), 'Color', colors(1, :), 'HandleVisibility', 'off');
end
plot(ax, x_step(:, 1), x_step(:, 2), 'k', 'LineWidth', 2, 'DisplayName', 'unit step');
axis(ax, [-0.36 0.36 -3.6 3.6]);
grid(ax, 'on');
xlabel(ax, 'position x [m]');
ylabel(ax, 'velocity v [m/s]');
title(ax, sprintf('Phase portrait (poles %s)', format_poles(results.msd.poles)));
legend(ax);
export_figure(fig, opts, 'mass_spring_damper');
end
