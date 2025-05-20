function sys = state_feedback_loop(plant, K, V)
%STATE_FEEDBACK_LOOP Closed loop u = V w - K x around a plant struct.
%   sys = STATE_FEEDBACK_LOOP(plant, K, V) returns an ss object with inputs [w; z]
%   (reference and the plant's disturbances) and outputs [y; u]:
%
%       x' = (A - B K) x + B V w + E z
%       y  = (C - D K) x + D V w
%       u  =      -K x +   V w
%
%   Pole placement and LQR designs share this structure; only K differs.

A = plant.A;
B = plant.B;
C = plant.C;
D = plant.D;
E = plant.E;
q = size(E, 2);

sys = ss(A - B * K, [B * V, E], [C - D * K; -K], ...
    [D * V, zeros(size(C, 1), q); V, zeros(size(K, 1), q)], ...
    'StateName', cellstr(plant.states), ...
    'InputName', [{'w'}, cellstr(plant.disturbances)], ...
    'OutputName', {'y', 'u'});
end
