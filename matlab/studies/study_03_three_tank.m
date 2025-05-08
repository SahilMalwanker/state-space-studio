function results = study_03_three_tank(opts)
%STUDY_03_THREE_TANK Three-tank level control: four controllers on one test profile.
%   results = STUDY_03_THREE_TANK() runs
%     1. structural analysis and open-loop responses (free draining, pump/leak profile)
%     2. pole placement + prefilter (dominant double pole from t_s = 200 s, zeta = 1)
%     3. PI state feedback (integrator state, poles -0.02 x2 and -0.2 x2)
%     4. LQR with Q = I, R = 2 and with Q = diag(10, 1, 10), R = 5 (+ prefilter)
%   and compares them on the same test: reference 30 -> 70 % at t = 500 s and a
%   leak z = -5 from t = 1000 s.
%   results = STUDY_03_THREE_TANK(opts) exports figures when opts.figure_dir is set.
%
%   Python counterpart: state_space_studio.scenarios.three_tank_study

if nargin < 1
    opts = struct();
end
opts = merge_params(struct('figure_dir', ""), opts);
colors = studio_colors();
plant = three_tank_model();
level_labels = ["h_1", "h_2", "h_3"];

%% 1. Structural properties and open-loop behaviour
results.report = structural_analysis(plant);
open_loop = ss(plant.A, [plant.B, plant.E], eye(3), zeros(3, 2));

t_free = (0:1:1200)';
[~, ~, x_free] = initial(open_loop, [60; 50; 40], t_free);
t_ol = (0:1:5000)';
ol_inputs = [step_signal(t_ol, 1500, 10, 25), step_signal(t_ol, 3000, 0, -5)];
x_ol = lsim(open_loop, ol_inputs, t_ol);

fig = figure('Name', 'Three-tank - open loop', 'Color', 'w', 'Position', [60 60 1250 450]);
layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(layout);
plot_states(ax, t_free, x_free, level_labels);
title(ax, 'Free response from (60, 50, 40) %');
xlabel(ax, 'time [s]');
ylabel(ax, 'level [%]');
legend(ax);
ax = nexttile(layout);
plot_states(ax, t_ol, x_ol, level_labels);
plot(ax, t_ol, ol_inputs(:, 1), '--', 'Color', colors(5, :), 'DisplayName', 'pump u');
plot(ax, t_ol, ol_inputs(:, 2), '--', 'Color', colors(4, :), 'DisplayName', 'disturbance z');
title(ax, 'Pump step at 1500 s, leak at 3000 s');
xlabel(ax, 'time [s]');
legend(ax, 'Location', 'northwest');
export_figure(fig, opts, 'three_tank_open_loop');

%% 2.-4. Controller designs
dominant = second_order_poles(200, 1);                 % double pole at -0.02
designs = struct('name', {}, 'loop', {}, 'K', {}, 'V', {});

K = acker(plant.A, plant.B, [dominant; 5 * dominant(1)]);
V = prefilter_gain(plant.A, plant.B, plant.C, K);
designs(end + 1) = struct('name', "Pole placement + prefilter", ...
    'loop', state_feedback_loop(plant, K, V), 'K', K, 'V', V);

gains = pi_state_feedback_design(plant.A, plant.B, plant.C, ...
    [dominant; 10 * dominant(1); 10 * dominant(1)], 1);
results.pi_gains = gains;
designs(end + 1) = struct('name', "PI state feedback", ...
    'loop', pi_state_feedback_loop(plant, gains), 'K', gains.Kx, 'V', gains.Kp);

weights = {eye(3), 2, "LQR Q = I, R = 2"; diag([10 1 10]), 5, "LQR Q = diag(10, 1, 10), R = 5"};
for k = 1:size(weights, 1)
    [K, ~, ~] = lqr(plant.A, plant.B, weights{k, 1}, weights{k, 2});
    V = prefilter_gain(plant.A, plant.B, plant.C, K);
    designs(end + 1) = struct('name', weights{k, 3}, ...
        'loop', state_feedback_loop(plant, K, V), 'K', K, 'V', V); %#ok<AGROW>
end

%% Common test profile
t = (0:0.5:1500)';
test_inputs = [step_signal(t, 500, 30, 70), step_signal(t, 1000, 0, -5)];
before_leak = find(t < 1000, 1, 'last');
after_step = t >= 500;

fig = figure('Name', 'Three-tank - controller comparison', 'Color', 'w', 'Position', [40 40 1250 720]);
layout = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_y = nexttile(layout);
plot(ax_y, t, test_inputs(:, 1), 'k--', 'LineWidth', 1.2, 'DisplayName', 'reference w');
hold(ax_y, 'on');
ax_u = nexttile(layout);
hold(ax_u, 'on');
order = [4 1 2 5];                                      % rose, blue, amber, violet
summary = table('Size', [numel(designs), 6], ...
    'VariableTypes', {'double', 'double', 'double', 'double', 'double', 'double'}, ...
    'VariableNames', {'settling_s', 'error_before_leak', 'error_after_leak', 'u_min', 'u_max', 'IAE'}, ...
    'RowNames', cellstr([designs.name]));
for k = 1:numel(designs)
    sys = designs(k).loop;
    y_u = lsim(sys(1:2, :), test_inputs, t);           % outputs y and u
    y = y_u(:, 1);
    u = y_u(:, 2);
    error_signal = test_inputs(:, 1) - y;
    window = t >= 500 & t < 1000;
    info = stepinfo(y(window) - y(find(window, 1)), t(window) - 500);
    summary{k, :} = [info.SettlingTime, error_signal(before_leak), error_signal(end), ...
        min(u), max(u), 0.5 * sum(abs(error_signal(after_step)))];
    color = colors(order(k), :);
    plot(ax_y, t, y, 'Color', color, 'LineWidth', 1.6, 'DisplayName', designs(k).name);
    plot(ax_u, t, u, 'Color', color, 'LineWidth', 1.4, 'DisplayName', designs(k).name);
end
patch(ax_y, [1000 1500 1500 1000], [0 0 80 80], colors(4, :), 'FaceAlpha', 0.06, ...
    'EdgeColor', 'none', 'HandleVisibility', 'off');
ylim(ax_y, [0 80]);
grid(ax_y, 'on');
ylabel(ax_y, 'tank-3 level y [%]');
title(ax_y, 'Reference step at 500 s, leak z = -5 from 1000 s (shaded)');
legend(ax_y, 'Location', 'northwest');
yline(ax_u, 0, 'LineStyle', ':', 'HandleVisibility', 'off');
yline(ax_u, 100, 'LineStyle', ':', 'HandleVisibility', 'off');
grid(ax_u, 'on');
xlabel(ax_u, 'time [s]');
ylabel(ax_u, 'pump command u [%]');
title(ax_u, 'Actuating signal (feasible range 0 ... 100 %)');
export_figure(fig, opts, 'three_tank_controllers');

results.designs = designs;
results.summary = summary;
fprintf('\nThree-tank controllers on the common test profile:\n');
disp(summary);
end
