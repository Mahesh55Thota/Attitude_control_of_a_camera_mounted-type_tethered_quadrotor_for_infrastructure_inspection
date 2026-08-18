function pos = tether_pos_func(u)
% TETHER_POS_FUNC Helper function for Simulink Tether Position Sensing Block
% Inputs: u = [alpha (rad); beta (rad); l (m)]
% Outputs: pos = [x (m); y (m); z (m)] (Eqs. 1-3 from Paper)

alpha = u(1);
beta  = u(2);
l     = u(3);

% Eq. (3): Altitude z calculation
num = l^2 * (cos(alpha))^2 * (cos(beta))^2;
den = (cos(alpha))^2 + (cos(beta))^2 - (cos(alpha))^2 * (cos(beta))^2;

if den <= 0
    z = -l;
else
    z = -sqrt(num / den);
end

% Eqs. (1)-(2): X and Y position calculation
x = z * tan(alpha);
y = z * tan(beta);

pos = [x; y; z];
end
