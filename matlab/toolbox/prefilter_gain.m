function V = prefilter_gain(A, B, C, K, D)
%PREFILTER_GAIN Static prefilter for zero steady-state error of u = V w - K x.
%   V = PREFILTER_GAIN(A, B, C, K) returns V = inv(-C (A - B K)^-1 B).
%   V = PREFILTER_GAIN(A, B, C, K, D) includes a direct feedthrough term.

if nargin < 5
    D = zeros(size(C, 1), size(B, 2));
end
closed_loop_dc = (C - D * K) * (-(A - B * K) \ B) + D;
V = inv(closed_loop_dc);
end
