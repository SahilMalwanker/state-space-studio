function results = study_02_drivetrain(opts)
%STUDY_02_DRIVETRAIN Three-mass torsional drivetrain: analysis and pole placement.
%   results = STUDY_02_DRIVETRAIN() runs the complete workflow:
%     1. structural analysis (rigid-body mode, torsional modes, Kalman ranks)
%     2. open-loop responses to initial twists, a motor torque step and load torques
%     3. pole placement (-10, -10, -100 x4) with Ackermann's formula plus a prefilter
%     4. five closed-loop test cases (reference step and load-torque steps)
%   results = STUDY_02_DRIVETRAIN(opts) exports figures when opts.figure_dir is set.
%
%   Python counterpart: state_space_studio.scenarios.drivetrain_study

if nargin < 1
    opts = struct();
end
opts = merge_params(struct('figure_dir', ""), opts);
colors = studio_colors();
plant = drivetrain_model();
angle_labels = ["\phi_1 (motor side)", "\phi_2 (middle)", "\phi_3 (load)"];
speed_labels = ["\omega_1", "\omega_2", "\omega_3"];

%% 1. Structural properties
results.report = structural_analysis(plant);
p = results.report.poles;
torsional = p(imag(p) > 1e-6);
fprintf('  torsional modes [rad/s]    : %s\n', mat2str(abs(torsional).', 4));
fprintf('  their damping ratios       : %s\n', mat2str((-real(torsional) ./ abs(torsional)).', 3));
fprintf('  rigid-body mode            : one pole at 0 (marginal stability)\n');

%% 2. Open-loop behaviour (all states as outputs; inputs [u Tz1 Tz2 Tz3])
open_loop = ss(plant.A, [plant.B, plant.E], eye(6), zeros(6, 4));
t = (0:5e-4:1)';
fig = figure('Name', 'Drivetrain - initial conditions', 'Color', 'w', 'Position', [60 60 1300 600]);
layout = tiledlayout(fig, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:3
    x0 = zeros(6, 1);
    x0(k) = 0.05;
    [~, ~, x] = initial(open_loop, x0, t);
    ax = nexttile(layout, k);
    plot_states(ax, t, x(:, 1:3), angle_labels);
    title(ax, sprintf('\\phi_%d(0) = 0.05 rad', k));
    ylabel(ax, 'angle [rad]');
    ax = nexttile(layout, k + 3);
    plot_states(ax, t, x(:, 4:6), speed_labels);
    xlabel(ax, 'time [s]');
    ylabel(ax, 'angular velocity [rad/s]');
end
legend(nexttile(layout, 1), 'Location', 'northeast');
export_figure(fig, opts, 'drivetrain_initial_conditions');

t_step = (0:2.5e-4:0.5)';
u_step = [10 * ones(size(t_step)), zeros(numel(t_step), 3)];
x_torque = lsim(open_loop, u_step, t_step);
x_tz2 = lsim(open_loop, repmat([0 0 1 0], numel(t), 1), t);
x_tz3 = lsim(open_loop, repmat([0 0 0 1], numel(t), 1), t);
results.open_loop_speed = x_torque(end, 4:6);

fig = figure('Name', 'Drivetrain - open-loop inputs', 'Color', 'w', 'Position', [60 60 1300 600]);
layout = tiledlayout(fig, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
cases = {t_step, x_torque, 'motor torque u = 10 N m'; ...
         t, x_tz2, 'load torque T_{z2} = 1 N m'; ...
         t, x_tz3, 'load torque T_{z3} = 1 N m'};
for k = 1:3
    ax = nexttile(layout, k);
    plot_states(ax, cases{k, 1}, cases{k, 2}(:, 1:3), angle_labels);
    title(ax, cases{k, 3});
    ylabel(ax, 'angle [rad]');
    ax = nexttile(layout, k + 3);
    plot_states(ax, cases{k, 1}, cases{k, 2}(:, 4:6), speed_labels);
    xlabel(ax, 'time [s]');
    ylabel(ax, 'angular velocity [rad/s]');
end
legend(nexttile(layout, 1), 'Location', 'northwest');
title(layout, 'Open loop: the rigid-body mode (pole at 0) lets the shaft spin up without bound');
export_figure(fig, opts, 'drivetrain_open_loop');

%% 3. Pole placement and prefilter
poles = [-10 -10 -100 -100 -100 -100];
K = acker(plant.A, plant.B, poles);
V = prefilter_gain(plant.A, plant.B, plant.C, K);
results.K = K;
results.V = V;
fprintf('\nPole placement: K = %s\n', mat2str(K, 6));
fprintf('Prefilter     : V = %.4f (equals the sum of the angle gains, %.4f)\n', V, sum(K(1:3)));
fprintf('Rule of thumb : t_s = 4 / 10 = 0.4 s for the dominant pair at -10;\n');
fprintf('                a critically damped double pole really needs about 5.83 / 10 s.\n');

%% 4. Closed-loop test cases (constant inputs applied at t = 0)
loop = state_feedback_loop(plant, K, V);
loop_states = ss(loop.A, loop.B, [eye(6); loop.C(2, :)], [zeros(6, 4); loop.D(2, :)]);
titles = ["reference w = 50 rad", "T_{z1} = 1 N m", "T_{z2} = 1 N m", "T_{z3} = 1 N m", ...
          "T_{z2} = 0.5 N m, T_{z3} = 1 N m"];
inputs = [50 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 0.5 1];   % columns [w Tz1 Tz2 Tz3]

fig = figure('Name', 'Drivetrain - state feedback', 'Color', 'w', 'Position', [40 40 1700 820]);
layout = tiledlayout(fig, 3, 5, 'TileSpacing', 'compact', 'Padding', 'compact');
results.final_angles = zeros(5, 3);
results.final_torque = zeros(5, 1);
for k = 1:5
    response = lsim(loop_states, repmat(inputs(k, :), numel(t), 1), t);
    results.final_angles(k, :) = response(end, 1:3);
    results.final_torque(k) = response(end, 7);

    ax = nexttile(layout, k);
    plot_states(ax, t, response(:, 1:3), angle_labels);
    title(ax, titles(k));
    ax = nexttile(layout, k + 5);
    plot_states(ax, t, response(:, 4:6), speed_labels);
    ax = nexttile(layout, k + 10);
    plot(ax, t, response(:, 7), 'Color', colors(5, :), 'LineWidth', 1.6);
    grid(ax, 'on');
    box(ax, 'off');
    xlabel(ax, 'time [s]');
    if k == 1
        info = stepinfo(response(:, 3), t, 50);
        results.tracking = info;
        ylabel(nexttile(layout, 1), 'angle [rad]');
        ylabel(nexttile(layout, 6), 'angular velocity [rad/s]');
        ylabel(ax, 'motor torque u [N m]');
    end
end
legend(nexttile(layout, 1), 'Location', 'southeast');
title(layout, sprintf('State feedback, poles -10 (x2) and -100 (x4), prefilter V = %.4f', V));
export_figure(fig, opts, 'drivetrain_state_feedback');

fprintf('Reference step: rise time %.3f s, settling time %.3f s, overshoot %.2f %%\n', ...
    results.tracking.RiseTime, results.tracking.SettlingTime, results.tracking.Overshoot);
disp(array2table([results.final_angles, results.final_torque], ...
    'VariableNames', {'phi1_1s', 'phi2_1s', 'phi3_1s', 'u_1s'}, 'RowNames', cellstr(titles)));
end
