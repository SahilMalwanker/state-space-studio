function s = step_signal(t, at, before, after)
%STEP_SIGNAL Column vector that switches from BEFORE to AFTER at time AT.
%   The new value applies from t = AT onwards (exact for lsim's zero-order hold when AT
%   lies on the time grid).

s = before + (after - before) * double(t(:) >= at - 1e-9 * max(1, abs(at)));
end
