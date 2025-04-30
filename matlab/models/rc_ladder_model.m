function plant = rc_ladder_model(params)
%RC_LADDER_MODEL Three cascaded RC low-pass stages driven by a source voltage.
%   plant = RC_LADDER_MODEL() returns a struct with fields A, B, C, D, E, name,
%   states and disturbances for
%
%       x' = A x + B u,   y = C x,   x = [v1 v2 v3]' (capacitor voltages, V)
%
%   plant = RC_LADDER_MODEL(params) overrides R (1x3, ohm) and/or C (1x3, F).
%
%   See also COUPLED_MIMO_MODEL, MASS_SPRING_DAMPER_MODEL.

if nargin < 1
    params = struct();
end
p = merge_params(struct('R', [4 4 4], 'C', [1 1 1]), params);
R = p.R;
Cap = p.C;

plant.name = "RC ladder network";
plant.A = [-(1/R(1) + 1/R(2)) / Cap(1),  1 / (R(2) * Cap(1)),           0;
            1 / (R(2) * Cap(2)),        -(1/R(2) + 1/R(3)) / Cap(2),   1 / (R(3) * Cap(2));
            0,                           1 / (R(3) * Cap(3)),          -1 / (R(3) * Cap(3))];
plant.B = [1 / (R(1) * Cap(1)); 0; 0];
plant.C = [0 0 1];
plant.D = 0;
plant.E = zeros(3, 0);
plant.states = ["v1" "v2" "v3"];
plant.disturbances = strings(1, 0);
plant.params = p;
end
