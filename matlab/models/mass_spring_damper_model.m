function plant = mass_spring_damper_model(params)
%MASS_SPRING_DAMPER_MODEL Translational oscillator  m x'' + b x' + c x = u.
%   State x = [position; velocity], output y = position.
%   plant = MASS_SPRING_DAMPER_MODEL(params) overrides m (kg), b (N s/m), c (N/m).
%
%   See also RC_LADDER_MODEL, COUPLED_MIMO_MODEL.

if nargin < 1
    params = struct();
end
p = merge_params(struct('m', 0.05, 'b', 1, 'c', 4), params);

plant.name = "Mass-spring-damper";
plant.A = [0, 1; -p.c / p.m, -p.b / p.m];
plant.B = [0; 1 / p.m];
plant.C = [1 0];
plant.D = 0;
plant.E = zeros(2, 0);
plant.states = ["x" "v"];
plant.disturbances = strings(1, 0);
plant.params = p;
end
