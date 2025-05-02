function build_simulink_models(out_dir)
%BUILD_SIMULINK_MODELS Generate the project's Simulink block diagrams from code.
%   BUILD_SIMULINK_MODELS() creates three models in matlab/simulink/build:
%
%     sc_drivetrain_state_feedback   plant built from integrators and gain blocks
%                                    (no State-Space block), u = V w - K x,
%                                    constant reference w and load torques z
%     sc_three_tank_state_feedback   same structure with a reference step and a leak
%                                    step (pole placement and LQR share it)
%     sc_three_tank_pi               PI state feedback, integrator on e = w - y
%
%   BUILD_SIMULINK_MODELS(out_dir) writes the .slx files to OUT_DIR instead.
%
%   Every block parameter is a workspace variable (A, B, C, D, E, K, V, Kp, Ki, Kpi, x0,
%   w, z, w_time, ...), so one diagram serves several designs. Generating the models
%   keeps them reviewable as text. RUN_SIMULINK_CROSSCHECK simulates them and compares
%   the result with lsim.
%
%   Requires Simulink.

if nargin < 1
    out_dir = fullfile(fileparts(mfilename('fullpath')), 'build');
end
if ~isfolder(out_dir)
    mkdir(out_dir);
end
build_state_feedback_model('sc_drivetrain_state_feedback', out_dir, 'constant', '1', '1e-4');
build_state_feedback_model('sc_three_tank_state_feedback', out_dir, 'step', '1500', '0.5');
build_pi_model('sc_three_tank_pi', out_dir);
fprintf('Simulink models written to %s\n', out_dir);
end

% -------------------------------------------------------------------------------------
function build_state_feedback_model(mdl, out_dir, source_kind, stop_time, step_size)
start_model(mdl, out_dir);
add_sources(mdl, source_kind);
add_gain(mdl, 'V', [110 100 150 130], 'V', 'Element-wise(K.*u)', 'right');
add_sum(mdl, 'sum_u', [190 103 214 127], '|+-', 'round');
add_plant(mdl);
add_gain(mdl, 'K', [410 260 460 290], 'K', 'Matrix(K*u) (u vector)', 'left');
add_logger(mdl, 'log_u', [250 300 320 330], 'u_log');

connect(mdl, 'w/1', 'V/1');
connect(mdl, 'V/1', 'sum_u/1');
connect(mdl, 'K/1', 'sum_u/2');
connect(mdl, 'sum_u/1', 'B/1');
connect(mdl, 'sum_u/1', 'D/1');
connect(mdl, 'sum_u/1', 'log_u/1');
connect(mdl, 'integrator/1', 'K/1');
finish_model(mdl, out_dir, stop_time, step_size);
end

% -------------------------------------------------------------------------------------
function build_pi_model(mdl, out_dir)
start_model(mdl, out_dir);
add_sources(mdl, 'step');
add_sum(mdl, 'sum_e', [90 103 114 127], '|+-', 'round');
add_block('simulink/Continuous/Integrator', [mdl '/integrator_e'], ...
    'Position', [130 160 160 190], 'InitialCondition', '0');
add_gain(mdl, 'Kp', [130 55 170 85], 'Kp', 'Element-wise(K.*u)', 'right');
add_gain(mdl, 'Ki', [175 160 210 190], 'Ki', 'Element-wise(K.*u)', 'right');
add_sum(mdl, 'sum_pi', [185 103 209 127], '+|+', 'round');
add_sum(mdl, 'sum_u', [225 103 249 127], '|+-', 'round');
add_plant(mdl);
add_gain(mdl, 'Kpi', [410 260 460 290], 'Kpi', 'Matrix(K*u) (u vector)', 'left');
add_logger(mdl, 'log_u', [250 300 320 330], 'u_log');
add_logger(mdl, 'log_e', [130 215 200 245], 'e_log');

connect(mdl, 'w/1', 'sum_e/1');
connect(mdl, 'C/1', 'sum_e/2');
connect(mdl, 'sum_e/1', 'Kp/1');
connect(mdl, 'sum_e/1', 'integrator_e/1');
connect(mdl, 'sum_e/1', 'log_e/1');
connect(mdl, 'integrator_e/1', 'Ki/1');
connect(mdl, 'Kp/1', 'sum_pi/1');
connect(mdl, 'Ki/1', 'sum_pi/2');
connect(mdl, 'sum_pi/1', 'sum_u/1');
connect(mdl, 'Kpi/1', 'sum_u/2');
connect(mdl, 'sum_u/1', 'B/1');
connect(mdl, 'sum_u/1', 'D/1');
connect(mdl, 'sum_u/1', 'log_u/1');
connect(mdl, 'integrator/1', 'Kpi/1');
finish_model(mdl, out_dir, '1500', '0.5');
end

% -------------------------------------------------------------------------------------
function add_plant(mdl)
% x' = A x + B u + E z,  y = C x + D u  built from gains, two sums and one integrator.
add_gain(mdl, 'B', [270 100 320 130], 'B', 'Matrix(K*u) (u vector)', 'right');
add_gain(mdl, 'E', [270 20 320 50], 'E', 'Matrix(K*u) (u vector)', 'right');
add_sum(mdl, 'sum_x', [350 90 370 140], '+++', 'rectangular');
add_block('simulink/Continuous/Integrator', [mdl '/integrator'], ...
    'Position', [400 100 440 130], 'InitialCondition', 'x0');
add_gain(mdl, 'A', [410 170 460 200], 'A', 'Matrix(K*u) (u vector)', 'left');
add_gain(mdl, 'C', [520 100 570 130], 'C', 'Matrix(K*u) (u vector)', 'right');
add_gain(mdl, 'D', [520 200 570 230], 'D', 'Element-wise(K.*u)', 'right');
add_sum(mdl, 'sum_y', [610 103 634 127], '|++', 'round');
add_logger(mdl, 'log_x', [520 30 590 60], 'x_log');
add_logger(mdl, 'log_y', [680 100 750 130], 'y_log');

connect(mdl, 'z/1', 'E/1');
connect(mdl, 'E/1', 'sum_x/1');
connect(mdl, 'B/1', 'sum_x/2');
connect(mdl, 'A/1', 'sum_x/3');
connect(mdl, 'sum_x/1', 'integrator/1');
connect(mdl, 'integrator/1', 'A/1');
connect(mdl, 'integrator/1', 'C/1');
connect(mdl, 'integrator/1', 'log_x/1');
connect(mdl, 'C/1', 'sum_y/1');
connect(mdl, 'D/1', 'sum_y/2');
connect(mdl, 'sum_y/1', 'log_y/1');
end

function add_sources(mdl, source_kind)
if strcmp(source_kind, 'constant')
    add_block('simulink/Sources/Constant', [mdl '/w'], 'Position', [30 100 70 130], 'Value', 'w');
    add_block('simulink/Sources/Constant', [mdl '/z'], 'Position', [30 20 70 50], 'Value', 'z');
else
    add_block('simulink/Sources/Step', [mdl '/w'], 'Position', [30 100 70 130], ...
        'Time', 'w_time', 'Before', 'w_before', 'After', 'w_after');
    add_block('simulink/Sources/Step', [mdl '/z'], 'Position', [30 20 70 50], ...
        'Time', 'z_time', 'Before', 'z_before', 'After', 'z_after');
end
end

function add_gain(mdl, name, position, gain, multiplication, orientation)
add_block('simulink/Math Operations/Gain', [mdl '/' name], 'Position', position, ...
    'Gain', gain, 'Multiplication', multiplication, 'Orientation', orientation);
end

function add_sum(mdl, name, position, inputs, shape)
add_block('simulink/Math Operations/Sum', [mdl '/' name], 'Position', position, ...
    'Inputs', inputs, 'IconShape', shape);
end

function add_logger(mdl, name, position, variable)
add_block('simulink/Sinks/To Workspace', [mdl '/' name], 'Position', position, ...
    'VariableName', variable, 'SaveFormat', 'Timeseries');
end

function connect(mdl, from, to)
add_line(mdl, from, to, 'autorouting', 'on');
end

function start_model(mdl, out_dir)
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
file = fullfile(out_dir, [mdl '.slx']);
if isfile(file)
    delete(file);
end
new_system(mdl);
end

function finish_model(mdl, out_dir, stop_time, step_size)
set_param(mdl, 'StopTime', stop_time, 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
    'FixedStep', step_size);
save_system(mdl, fullfile(out_dir, [mdl '.slx']));
close_system(mdl, 0);
end
