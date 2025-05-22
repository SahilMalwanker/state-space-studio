function report = structural_analysis(plant, verbose)
%STRUCTURAL_ANALYSIS Eigenvalues, stability class and Kalman ranks of a plant.
%   report = STRUCTURAL_ANALYSIS(plant) returns a struct with fields poles, stability,
%   ctrb_rank, obsv_rank, controllable and observable, and prints a short summary.
%   STRUCTURAL_ANALYSIS(plant, false) suppresses the printout.
%
%   Repeated imaginary-axis eigenvalues require a separate Jordan-structure check.

if nargin < 2
    verbose = true;
end
A = plant.A;
n = size(A, 1);

report.poles = eig(A);
report.stability = classify_stability(report.poles);
report.ctrb_rank = rank(ctrb(A, plant.B));
report.obsv_rank = rank(obsv(A, plant.C));
report.controllable = report.ctrb_rank == n;
report.observable = report.obsv_rank == n;

if verbose
    fprintf('\n%s\n', plant.name);
    fprintf('  poles      : %s\n', format_poles(report.poles));
    fprintf('  stability  : %s\n', report.stability);
    fprintf('  rank Qc = %d, rank Qo = %d (n = %d)\n', report.ctrb_rank, report.obsv_rank, n);
end
end

function label = classify_stability(poles)
tol = 1e-9;
if any(real(poles) > tol)
    label = "unstable";
    return
end
on_axis = poles(abs(real(poles)) <= tol);
label = "asymptotically stable";
if ~isempty(on_axis)
    label = "marginally stable";
    for k = 1:numel(on_axis)
        if sum(abs(on_axis - on_axis(k)) <= 1e-6) > 1
            label = "undetermined (repeated imaginary-axis poles)";
        end
    end
end
end
