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

function [A_n, current_V, current_P, current_q, current_omega] = INS(f_hat, gyro_hat, dt, past_omega, past_q, g_n, past_V, past_P)
%Intergrate quat
w_x_hat = gyro_hat(1, 1);
w_y_hat = gyro_hat(2, 1);
w_z_hat = gyro_hat(3, 1);

current_omega = [0  -w_x_hat -w_y_hat -w_z_hat;
                 w_x_hat  0  w_z_hat  -w_y_hat;
                 w_y_hat  -w_z_hat  0  w_x_hat;
                 w_z_hat  w_y_hat  -w_x_hat  0];

%current_Quat계산
current_q = (eye(4) + 3 / 4 * current_omega * dt - 1 / 4 * past_omega * dt - 1/6 * norm(gyro_hat)^2 * eye(4) * dt^2 - (1 / 24) * (current_omega) * (past_omega) * dt^2 - (1 / 48) * norm(gyro_hat)^2 * current_omega * dt^3) * past_q;

%quat -> DCM (body frame -> navigation frame)
C_b_n = quat2dcm(current_q);

%Accelerate
A_n = C_b_n * f_hat + [0; 0; g_n];
%Velocity
current_V = A_n * dt + past_V;
%Positon
current_P = current_V * dt + past_P;

end