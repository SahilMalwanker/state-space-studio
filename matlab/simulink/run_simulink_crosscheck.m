function report = run_simulink_crosscheck()
%RUN_SIMULINK_CROSSCHECK Simulate the generated Simulink models and compare with lsim.
%   report = RUN_SIMULINK_CROSSCHECK() builds the models if necessary, runs
%     - the drivetrain with pole placement and a 50 rad reference step,
%     - the three-tank process with pole placement + prefilter,
%     - the three-tank process with PI state feedback,
%   and returns the largest absolute deviation between the Simulink signals and the
%   lsim solution of the equivalent closed-loop state-space model.
%
%   Requires Simulink and the Control System Toolbox.

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(fullfile(root, 'models'), fullfile(root, 'toolbox'));
build_dir = fullfile(here, 'build');
if ~isfile(fullfile(build_dir, 'sc_three_tank_pi.slx'))
    build_simulink_models(build_dir);
end
addpath(build_dir);

%% Drivetrain: reference step w = 50 rad
plant = drivetrain_model();
K = acker(plant.A, plant.B, [-10 -10 -100 -100 -100 -100]);
V = prefilter_gain(plant.A, plant.B, plant.C, K);
in = Simulink.SimulationInput('sc_drivetrain_state_feedback');
in = with_plant(in, plant);
in = in.setVariable('K', K);
in = in.setVariable('V', V);
in = in.setVariable('w', 50);
in = in.setVariable('z', [0; 0; 0]);
in = in.setVariable('x0', zeros(6, 1));
out = sim(in);
t = out.x_log.Time;
loop = state_feedback_loop(plant, K, V);
x_ref = lsim(ss(loop.A, loop.B, eye(6), zeros(6, 4)), repmat([50 0 0 0], numel(t), 1), t);
report.drivetrain_states = max(abs(out.x_log.Data - x_ref), [], 'all');

%% Three-tank: common test profile (reference step at 500 s, leak from 1000 s)
plant = three_tank_model();
dominant = second_order_poles(200, 1);

K = acker(plant.A, plant.B, [dominant; 5 * dominant(1)]);
V = prefilter_gain(plant.A, plant.B, plant.C, K);
in = Simulink.SimulationInput('sc_three_tank_state_feedback');
in = with_test_profile(with_plant(in, plant));
in = in.setVariable('K', K);
in = in.setVariable('V', V);
in = in.setVariable('x0', zeros(3, 1));
out = sim(in);
t = out.y_log.Time;
loop = state_feedback_loop(plant, K, V);
y_ref = lsim(loop(1, :), test_inputs(t), t);
report.three_tank_state_feedback = max(abs(out.y_log.Data - y_ref));

gains = pi_state_feedback_design(plant.A, plant.B, plant.C, ...
    [dominant; 10 * dominant(1); 10 * dominant(1)], 1);
in = Simulink.SimulationInput('sc_three_tank_pi');
in = with_test_profile(with_plant(in, plant));
in = in.setVariable('Kp', gains.Kp);
in = in.setVariable('Ki', gains.Ki);
in = in.setVariable('Kpi', gains.Kpi);
in = in.setVariable('x0', zeros(3, 1));
out = sim(in);
t = out.y_log.Time;
loop = pi_state_feedback_loop(plant, gains);
y_ref = lsim(loop(1, :), test_inputs(t), t);
report.three_tank_pi = max(abs(out.y_log.Data - y_ref));

fprintf('Largest |Simulink - lsim| deviation per model:\n');
disp(report);
end

function in = with_plant(in, plant)
in = in.setVariable('A', plant.A);
in = in.setVariable('B', plant.B);
in = in.setVariable('C', plant.C);
in = in.setVariable('D', plant.D);
in = in.setVariable('E', plant.E);
end

function in = with_test_profile(in)
in = in.setVariable('w_time', 500);
in = in.setVariable('w_before', 30);
in = in.setVariable('w_after', 70);
in = in.setVariable('z_time', 1000);
in = in.setVariable('z_before', 0);
in = in.setVariable('z_after', -5);
end

function u = test_inputs(t)
u = [step_signal(t, 500, 30, 70), step_signal(t, 1000, 0, -5)];
end
