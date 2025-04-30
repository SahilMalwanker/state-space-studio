function plant = three_tank_model(params)
%THREE_TANK_MODEL Three coupled liquid tanks with a pump into tank 1.
%   plant = THREE_TANK_MODEL() returns the linearised model
%
%       x' = A x + B u + E z,   y = C x
%
%   with levels x = [h1 h2 h3]' (percent), pump command u (percent), disturbance z
%   entering tank 1 (a negative value models a leak) and y = h3.
%
%   plant = THREE_TANK_MODEL(params) overrides the flow coefficients k1, k2, k3 (drains),
%   k12, k23 (valves), kin (pump gain) and kz (disturbance gain), all in 1/s.
%
%   See also DRIVETRAIN_MODEL, PI_STATE_FEEDBACK_DESIGN.

if nargin < 1
    params = struct();
end
p = merge_params(struct('k1', 0.002, 'k2', 0.004, 'k3', 0.006, 'k12', 0.077, ...
    'k23', 0.081, 'kin', 0.034, 'kz', 0.06), params);

plant.name = "Three-tank liquid-level process";
plant.A = [-(p.k1 + p.k12),   p.k12,                      0;
            p.k12,           -(p.k12 + p.k23 + p.k2),     p.k23;
            0,                p.k23,                     -(p.k23 + p.k3)];
plant.B = [p.kin; 0; 0];
plant.E = [p.kz; 0; 0];
plant.C = [0 0 1];
plant.D = 0;
plant.states = ["h1" "h2" "h3"];
plant.disturbances = "z";
plant.params = p;
end
