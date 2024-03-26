% The MIT License
% 
% Copyright (c) 2021 Hoang Viet Do
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
% THE SOFTWARE.

function state_dot = droneModel(state, g, t_B, T, m, I_xx, I_yy, I_zz)
    phi = state(7);
    theta = state(8);
    psi = state(9);
    phi_dot = state(10);
    theta_dot = state(11);
    psi_dot = state(12);

    % Attitude
    c11 = 0;
    c12 = (I_yy - I_zz) * (theta_dot * cos(phi) * sin(phi) + psi_dot * sin(phi)^2 * cos(theta)) + (I_zz - I_yy) * psi_dot * cos(phi)^2 * cos(theta) - I_xx * psi_dot * cos(theta);
    c13 = (I_zz - I_yy) * psi_dot * cos(phi) * sin(phi) * cos(theta)^2;
    c21 = (I_zz - I_yy) * (theta_dot * cos(phi) * sin(phi) + psi_dot * sin(phi)^2 * cos(theta)) + (I_yy - I_zz) * psi_dot * cos(phi)^2 * cos(theta) + I_xx * psi_dot * cos(theta);
    c22 = (I_zz - I_yy) * phi_dot * cos(phi) * sin(phi);
    c23 = -I_xx * psi_dot * sin(theta) * cos(theta) + I_yy * psi_dot * sin(phi)^2 * sin(theta) * cos(theta) + I_zz * psi_dot * cos(phi)^2 * sin(theta) * cos(theta);
    c31 = (I_yy - I_zz) * psi_dot * cos(theta)^2 * sin(phi) * cos(phi) - I_xx * theta_dot * cos(theta);
    c32 = (I_zz - I_yy) * (theta_dot * cos(phi) * sin(phi) * sin(theta) + phi_dot * sin(phi)^2 * cos(theta)) + (I_yy - I_zz) * phi_dot * cos(phi)^2 * cos(theta) + I_xx * psi_dot * sin(theta) * cos(theta) - I_yy * psi_dot * sin(phi)^2 * sin(theta) * cos(theta) - I_zz * psi_dot * cos(phi)^2 * sin(theta) * cos(theta);
    c33 = (I_yy - I_zz) * phi_dot * cos(phi) * sin(phi) * cos(theta)^2 - I_yy * theta_dot * sin(phi)^2 * cos(theta) * sin(theta) - I_zz * theta_dot * cos(phi)^2 * cos(theta) * sin(theta) + I_xx * theta_dot * cos(theta) * sin(theta);

    C = [c11 c12 c13;c21 c22 c23;c31 c32 c33]; % equation (19);

    j11 = I_xx;
    j12 = 0;
    j13 = -I_xx * sin(theta);
    j21 = 0;
    j22 = I_yy * cos(phi)^2 + I_zz * sin(phi)^2;
    j23 = (I_yy - I_zz) * cos(phi) * sin(phi) * cos(theta);
    j31 = -I_xx * sin(theta);
    j32 = (I_yy - I_zz) * cos(phi) * sin(phi) * cos(theta);
    j33 = I_xx * sin(theta)^2 + I_yy * sin(phi)^2 * cos(theta)^2 + I_zz * cos(phi)^2 * cos(theta)^2;

    J = [j11, j12, j13;...
        j21, j22, j23;...
        j31, j32, j33]; % equation (16);

    J_inv=inv(J);

    eta_dot = [phi_dot;theta_dot;psi_dot];
    eta_dot_dot = J_inv * (t_B - C * eta_dot); % equation (20)

    % Position
    epsilon_dot_dot = -[0; 0; g] + T/m * [cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);...
        sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);...
        cos(theta) * cos(phi)];

    state_dot = zeros(12, 1);

    state_dot(1:3) = state(4:6);
    state_dot(4:6) = epsilon_dot_dot;
    state_dot(7:9) = state(10:12);
    state_dot(10:12) = eta_dot_dot;
end