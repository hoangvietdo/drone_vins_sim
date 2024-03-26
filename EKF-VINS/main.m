%%
clear;
close all;
clc;

%% Load Data
addpath('../Tools')
load('../Datasets/drone_IMU_camera.mat')

%% Parameter
g_n = 9.8;
dt = 0.01;
I_3 = eye(3);
Z_3 = zeros(3,3);
r2d = 180 / pi;
d2r = 1 / r2d;

Est.Position(:, 1) = gT.Position(:, 1);
Est.Velocity(:, 1) = gT.Velocity(:, 1);
Est.BiasAccel(:, 1) = zeros(3, 1);
Est.BiasGyro(:, 1) = zeros(3, 1);
Est.Euler(:, 1) = gT.Euler(:, 1);

factor = IMU.Hz / Camera.Hz;

%% Initialization
init_q = euler2quat(gT.Euler(:, 1));
init_position = gT.Position(:, 1);
init_vel = gT.Velocity(:, 1);
init_bias_accel_hat = [0; 0; 0];
init_bias_gyro_hat = [0; 0; 0];

init_P_posteri = eye(15) * 1e-6;
init_bias_accel_true = gT.BiasAccel(:, 1); %groundtruth
init_bias_gyro_true = gT.BiasGyro(:, 1);
init_P_accel_bias = (init_bias_accel_true - init_bias_accel_hat).^2;
init_P_gyro_bias = (init_bias_gyro_true - init_bias_gyro_hat).^2;
init_P_posteri(10:12, 10:12) = diag(init_P_accel_bias);
init_P_posteri(13:15, 13:15) = diag(init_P_gyro_bias);

vel_rw = IMU.Noise.Accel^2 * ones(3,1);
ang_rw = IMU.Noise.Gyro^2 * ones(3,1);
ba_rw = 1e-6 * ones(3,1);
bg_rw = 1e-6 * ones(3,1);
Q = diag([vel_rw;ang_rw;ba_rw;bg_rw]); %12x12

%% Add image noise
for i = 1:1:length(Camera.FeatureMeasurement)
    for j = 1:1:length(Camera.FeatureMeasurement(:, :, i))
        if ~isnan(Camera.FeatureMeasurement(1, j, i))%i = time, j = image
            Camera.Measurement(:, j, i) = Camera.FeatureMeasurement(:, j, i) + randn(2, 1);
        else
            Camera.Measurement(:, j, i) = Camera.FeatureMeasurement(:, j, i);
        end
    end
end

%% Add noise to feature position w.r.t the global frame

Camera.P_f_G_noise = Camera.P_f_G + 0.1 * ones(3, length(Camera.P_f_G));

%% Main EKF for loop
usingCamera = 1; %0 : not using Camera (= only IMU)

for i = 1:1:length(gT.Time)
    if(i==1)
        past_q = init_q;
        past_vel = init_vel;
        past_position = init_position;

        bias_accel_hat = init_bias_accel_hat;
        bias_gyro_hat = init_bias_gyro_hat;

        P_past_posteri = init_P_posteri;
    end

    % Bias compensation
    f_hat = IMU.Accel(:, i) - bias_accel_hat;
    gyro_hat = IMU.Gyro(:, i) - bias_gyro_hat;

    if(i==1)
        past_omega = angular2omega(gyro_hat);
    else
        past_omega = current_omega;
    end

    %INS - Navigation Mechanization
    [a_n, current_vel, current_position, q_current, current_omega] = INS(f_hat, gyro_hat, dt, past_omega, past_q, g_n, past_vel, past_position);
    past_q = q_current;
    past_vel = current_vel;
    past_position = current_position;

    %% EKF
    % Time Update / Prediction
    C_n_b = quat2dcm(q_current);
    F_I = [I_3, I_3 * dt, Z_3, Z_3, Z_3;
        Z_3, I_3, skewSymmetricMatrix(C_n_b * f_hat) * dt, -C_n_b * dt, Z_3;
        Z_3, Z_3, I_3, Z_3, C_n_b * dt;
        Z_3, Z_3, Z_3, I_3, Z_3;
        Z_3, Z_3, Z_3, Z_3, I_3];

    G = [Z_3, Z_3, Z_3, Z_3 ;
        C_n_b, Z_3, Z_3, Z_3 ;
        Z_3, C_n_b, Z_3, Z_3 ;
        Z_3, Z_3, I_3, Z_3;
        Z_3, Z_3, Z_3, I_3];%15x12

    P_current_priori = F_I * P_past_posteri * F_I' + G * Q * dt * G';
    P_current_priori = (P_current_priori + P_current_priori') / 2;

    % Measurement Update / Update
    if usingCamera == 1
        if mod(i, factor) == 0
            n = floor(i / factor);
            hash_map = find(~isnan(Camera.Measurement(1, :, n)));

            if ~isempty(hash_map)
                y_meas = [];
                y_hat = [];
                H = [];

                number_of_deteted_feature = length(hash_map);
%                 P_aug = zeros(15 + number_of_deteted_feature * 3, 15 + number_of_deteted_feature * 3);

                for ii = 1:1:number_of_deteted_feature
                    foo = Camera.FeatureMeasurement(:, hash_map(ii), n);
                    y_meas = [y_meas; foo];

                    C_I_C = Camera.Extrinsic(1:3, 1:3)';
                    p_C_I = Camera.Extrinsic(1:3, 4);
                    enu2ned = [1 0 0; 0 -1 0; 0 0 -1]; % body ENU (same as forward-right-up) to body NED (same as forward-left-down)
                    C_G_I = quat2dcm(q_current)';
                    p_f_G = enu2ned * Camera.P_f_G_noise(:, hash_map(ii));
                    p_I_G = current_position;
                    p_f_C = C_I_C * C_G_I * (p_f_G - p_I_G);

                    foo = Camera.Intrinsic * p_f_C; % project 3D (X,Y,Z) to 2D homogeneous (u, v, w)
                    y_hat = [y_hat; [foo(1) / foo(3); foo(2) / foo(3)]]; % convert 2D homogeneous coordinate to 2D cartersian coordinate [u_,v_] = [u / w, v / w]

                    X = p_f_C(1);
                    Y = p_f_C(2);
                    Z = p_f_C(3);
                    G = (Camera.Intrinsic(1, 1) / Z) * [1, 0, -X / Z;...
                        0, 1, -Y / Z]; % Appendix C in my thesis, equation C.3

%                     H_j = G * C_I_C * C_G_I;
                    H_x = G * C_I_C * C_G_I * [-I_3, Z_3, -skewSymmetricMatrix(p_f_G - current_position), Z_3, Z_3]; % Appendix C in my thesis, equation C.8

                    % H = [H; [H_x, H_j]];
                    H = [H; H_x]; % stacking the feature measurement, 1 feature -> H = 2x15, 10 features -> H = 20 x 15
                end

                R = diag(ones(number_of_deteted_feature * 2 , 1));

                R_inv = inv(R);
                P_inv = inv(P_current_priori);
                Kalman_gain = inv(H' * R_inv * H + P_inv) * H' * R_inv;
%                 current_state = [current_position; current_vel; quat2euler(q_current); bias_accel_hat; bias_gyro_hat];

                r = y_hat - y_meas;
                delta_x_posteri = Kalman_gain * r;
                P_current_posteri = (eye(15) - Kalman_gain * H) * P_current_priori * (eye(15) - Kalman_gain * H)' + Kalman_gain * R * Kalman_gain';
                P_current_posteri = (P_current_posteri + P_current_posteri') / 2;
                P_past_posteri = P_current_posteri;

                % Correction
                current_position = past_position - delta_x_posteri(1:3);
                current_vel = past_vel - delta_x_posteri(4:6);
                Cbn_cur = (eye(3) + skewSymmetricMatrix(delta_x_posteri(7:9))) * quat2dcm(past_q);
                q_current = dcm2quat(Cbn_cur);
                bias_accel_hat = bias_accel_hat - delta_x_posteri(10:12);
                bias_gyro_hat = bias_gyro_hat - delta_x_posteri(13:15);

                past_position = current_position;
                past_q = q_current;
                past_vel = current_vel;
            else
                P_past_posteri = P_current_priori;
            end
        else
            P_past_posteri = P_current_priori;
        end
    else
        P_past_posteri = P_current_priori;
    end

    % Saving
    Est.Position(:, i) = current_position;
    Est.Euler(:, i) = quat2euler(q_current);
    Est.Velocity(:, i) = current_vel;
    Est.BiasGyro(:, i) = bias_gyro_hat;
    Est.BiasAccel(:, i) = bias_accel_hat;
end

%% Plotting
arglist = {'Marker', 'o', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'LineStyle', 'none'};

figure()
title('3D Trajectory Estimation');
plot3(gT.Position(1,:), gT.Position(2,:), gT.Position(3,:), 'b', 'LineWidth', 4, 'DisplayName', 'Ground Truth'); grid on; hold on;
plot3(Est.Position(1,:), -Est.Position(2,:), -Est.Position(3,:), 'r', 'LineWidth', 4, 'DisplayName', 'EKF-VINS');
plot3(Camera.P_f_G(1, :), Camera.P_f_G(2, :), Camera.P_f_G(3, :), arglist{:}, 'DisplayName', 'Landmarks')
legend;
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

