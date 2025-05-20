function p = second_order_poles(settling_time, damping, criterion)
%SECOND_ORDER_POLES Dominant pole pair from a settling-time target.
%   p = SECOND_ORDER_POLES(ts, zeta) returns the roots of
%   s^2 + 2 zeta wn s + wn^2 with wn = 4 / (zeta ts)  (2 % rule of thumb).
%   p = SECOND_ORDER_POLES(ts, zeta, criterion) replaces the factor 4.
%
%   The roots are computed in closed form, so a critically damped pair is returned as an
%   exact double pole (roots() would split it by round-off).

if nargin < 3
    criterion = 4;
end
wn = criterion / (damping * settling_time);
sigma = damping * wn;
if damping >= 1
    spread = wn * sqrt(damping^2 - 1);
    p = [-sigma - spread; -sigma + spread];
else
    wd = wn * sqrt(1 - damping^2);
    p = [-sigma - 1i * wd; -sigma + 1i * wd];
end
end
