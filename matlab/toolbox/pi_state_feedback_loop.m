function sys = pi_state_feedback_loop(plant, gains)
%PI_STATE_FEEDBACK_LOOP Closed loop u = Kp e + Ki x_i - Kpi x with x_i' = e = w - y.
%   sys = PI_STATE_FEEDBACK_LOOP(plant, gains) uses the gains from
%   PI_STATE_FEEDBACK_DESIGN and returns an ss object with states [x; xi],
%   inputs [w; z] and outputs [y; u; e].

if any(plant.D(:) ~= 0)
    error('stateSpaceStudio:properPlant', 'PI state feedback assumes a strictly proper plant (D = 0).');
end
A = plant.A;
B = plant.B;
C = plant.C;
E = plant.E;
q = size(E, 2);
Kx = gains.Kpi + gains.Kp * C;      % state part of the augmented gain

Acl = [A - B * Kx, gains.Ki * B; -C, 0];
Bcl = [gains.Kp * B, E; 1, zeros(1, q)];
Ccl = [C, 0; -Kx, gains.Ki; -C, 0];
Dcl = [0, zeros(1, q); gains.Kp, zeros(1, q); 1, zeros(1, q)];

sys = ss(Acl, Bcl, Ccl, Dcl, ...
    'StateName', [cellstr(plant.states), {'xi'}], ...
    'InputName', [{'w'}, cellstr(plant.disturbances)], ...
    'OutputName', {'y', 'u', 'e'});
end
