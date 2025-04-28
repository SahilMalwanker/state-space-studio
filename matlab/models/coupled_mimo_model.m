function plant = coupled_mimo_model()
%COUPLED_MIMO_MODEL Two-input third-order benchmark system.
%   The first state is a decoupled real mode (pole -3) that drives a lightly damped
%   pair (poles -1 +/- 4j). Output: y = x3.
%
%   See also RC_LADDER_MODEL, MASS_SPRING_DAMPER_MODEL.

plant.name = "Coupled two-input system";
plant.A = [-3   0   0;
            0  -2   1;
           10 -17   0];
plant.B = [4 0;
           0 1;
           0 0];
plant.C = [0 0 1];
plant.D = [0 0];
plant.E = zeros(3, 0);
plant.states = ["x1" "x2" "x3"];
plant.disturbances = strings(1, 0);
end
