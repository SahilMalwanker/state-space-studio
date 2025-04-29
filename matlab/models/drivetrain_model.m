function plant = drivetrain_model(params)
%DRIVETRAIN_MODEL Three inertias coupled by torsional springs, driven by a motor.
%   plant = DRIVETRAIN_MODEL() returns the model
%
%       x' = A x + B u + E z,   y = C x
%
%   with x = [phi1 phi2 phi3 omega1 omega2 omega3]' (rad, rad/s), u = motor torque T1
%   (N m), z = [Tz1 Tz2 Tz3]' load torques (N m) and y = phi3 (rad).
%
%   The matrices are assembled from the inertia, damping and stiffness matrices of
%   M phi'' + Dv phi' + Ks phi = [1;0;0] T1 + Tz.
%
%   plant = DRIVETRAIN_MODEL(params) overrides J (1x3), d (1x3), k12 and/or k23.
%
%   See also THREE_TANK_MODEL, STATE_FEEDBACK_LOOP.

if nargin < 1
    params = struct();
end
p = merge_params(struct('J', [2e-3 0.4e-3 0.3e-3], 'd', [8e-3 7.5e-3 7.5e-3], ...
    'k12', 1.5, 'k23', 1.5), params);

M_inv = diag(1 ./ p.J);
Dv = diag(p.d);
Ks = [ p.k12,  -p.k12,          0;
      -p.k12,   p.k12 + p.k23, -p.k23;
       0,      -p.k23,          p.k23];

plant.name = "Three-mass torsional drivetrain";
plant.A = [zeros(3), eye(3); -M_inv * Ks, -M_inv * Dv];
plant.E = [zeros(3); M_inv];
plant.B = plant.E(:, 1);            % the motor drives the first inertia
plant.C = [0 0 1 0 0 0];
plant.D = 0;
plant.states = ["phi1" "phi2" "phi3" "omega1" "omega2" "omega3"];
plant.disturbances = ["Tz1" "Tz2" "Tz3"];
plant.params = p;
end
