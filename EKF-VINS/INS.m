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