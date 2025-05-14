function gains = pi_state_feedback_design(A, B, C, poles, alpha)
%PI_STATE_FEEDBACK_DESIGN State feedback with integral action for a SISO plant.
%   gains = PI_STATE_FEEDBACK_DESIGN(A, B, C, poles) augments the plant with the
%   integrator x_i' = w - y, places the n+1 closed-loop poles with Ackermann's formula
%   and splits the result into the control law
%
%       u = Kp e + Ki x_i - Kpi x,   e = w - y.
%
%   Kp = alpha / G(0) is a free proportional term on the error (alpha = 1 by default);
%   Kpi = Kx - Kp C keeps the closed-loop poles exactly where they were placed.

if nargin < 5
    alpha = 1;
end
n = size(A, 1);
A_aug = [A, zeros(n, 1); -C, 0];
B_aug = [B; 0];
K_aug = acker(A_aug, B_aug, poles);

gains.K_aug = K_aug;
gains.Kx = K_aug(1:n);
gains.Ki = -K_aug(n + 1);
gains.Kp = alpha / (-C * (A \ B));
gains.Kpi = gains.Kx - gains.Kp * C;
gains.poles = poles;
gains.alpha = alpha;
end
