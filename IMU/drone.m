clear;
close all;
clc;

dt = 0.01; %randomly select, normal value 100 hz ~ 0.01 s

t = 0:dt:30;

% 8 shape
% a = 0.8;
% b = 0.7;

% circle 
a = 3;
b = 0.5;

g = 9.81; %m/s

% Drone parameters
m = 0.032; % drone's weigth
I_xx = 2.3951e-5;
I_yy = 2.3951e-5;
I_zz = 3.2347e-5;

% PD controller parameters(table 3)
K_x_P = 5;
K_x_D = 3;
K_x_DD = 0;

K_y_P = 5;
K_y_D = 3;
K_y_DD = 0;

K_z_P = 5;
K_z_D = 3;
K_z_DD = 0;

K_phi_P = 6;
K_phi_I = 3;
K_phi_D = 0;

K_theta_P = 6;
K_theta_I = 3;
K_theta_D = 0;

K_psi_P = 6;
K_psi_I = 1;
K_psi_D = 0.35;

K_phi_dot_P = 250;
K_phi_dot_I = 500;
K_phi_dot_D = 2.5;

K_theta_dot_P = 250;
K_theta_dot_I = 500;
K_theta_dot_D = 2.5;

K_psi_dot_P = 120;
K_psi_dot_I = 16.7;
K_psi_dot_D = 0;

%%
%initialize
x = 0;
y = 0;
z = 0;
x_dot = 0;
y_dot = 0;
z_dot = 0;
x_dot_dot = 0;
y_dot_dot = 0;
z_dot_dot = 0;

theta = 0;
phi = 0;
psi = 0;
phi_dot = 0;
theta_dot = 0;
psi_dot = 0;

e_phi_int_prev = 0;
e_theta_int_prev = 0;
e_psi_int_prev = 0;

e_phi_dot_int_prev = 0;
e_theta_dot_int_prev = 0;
e_psi_dot_int_prev = 0;

x_plot = zeros(12, length(t));
x_d_plot = zeros(6, length(t));

for i = 1:1:length(t)
    %% Tracking + Hovering Control(Stability)
    % Trajectory
% 8-shape trajectory
%     x_d = a * sin(b * t(i));
%     y_d = a * sin(b * t(i)) * cos(b * t(i));
%     z_d = 2;
% 
%     x_d_dot = a * b* cos(b * t(i));
%     y_d_dot = a * b * cos(b * t(i)) * cos(b * t(i)) - a * b * sin(b * t(i)) * sin(b * t(i));
%     z_d_dot = 0;
% 
%     x_d_dot_dot = -a * b^2 * sin(b * t(i));
%     y_d_dot_dot = -4 * a * b^2 * cos(b * t(i)) * sin(b * t(i));
%     z_d_dot_dot = 0;

    % Circle trajectory
    x_d = a * sin(b * t(i));
    y_d = a * cos(b * t(i));
    z_d = 2;

    x_d_dot = a * b * cos(b * t(i));
    y_d_dot = -a * b * sin(b * t(i));
    z_d_dot = 0;

    x_d_dot_dot = -a * b * b * sin(b * t(i));
    y_d_dot_dot = -a * b * b * cos(b * t(i));
    z_d_dot_dot = 0;

    % calculate the PD controller for position (equation 29)
    d_x = K_x_P * (x_d - x) + K_x_D * (x_d_dot - x_dot) + K_x_DD * (x_d_dot_dot - x_dot_dot);
    d_y = K_y_P * (y_d - y) + K_y_D * (y_d_dot - y_dot) + K_y_DD * (y_d_dot_dot - y_dot_dot);
    d_z = K_z_P * (z_d - z) + K_z_D * (z_d_dot - z_dot) + K_z_DD * (z_d_dot_dot - z_dot_dot);

    % calculate the desired angle (equation 26)
    phi_d = asin((d_x * sin(psi) - d_y * cos(psi))/sqrt(d_x^2 + d_y^2 + (d_z + g)^2));
    theta_d = atan2(d_x * cos(psi) + d_y * sin(psi), d_z + g);
    T = m * (d_x * (sin(theta) * cos(psi) * cos(phi) + sin(psi) * sin(phi)) + d_y * ( sin(theta) * sin(psi) * cos(phi) - cos(psi) * sin(phi)) + (d_z + g) * cos(theta) * cos(phi));
    psi_d = 0;

    % calculate the error (equation 23)
    e_phi = phi_d - phi;
    e_theta = theta_d - theta;
    e_psi = psi_d - psi;

    %% Changed !
    % calculate the PID controller for attitude
    [phi_d_2, e_phi_int] = PID(e_phi, e_phi_int_prev, K_phi_P, K_phi_I, K_phi_D, dt);
    [theta_d_2, e_theta_int] = PID(e_theta, e_theta_int_prev, K_theta_P, K_theta_I, K_theta_D, dt);
    [psi_d_2, e_psi_int] = PID(e_psi, e_psi_int_prev, K_psi_P, K_psi_I, K_psi_D, dt);

    e_phi_dot = phi_d_2 - phi_dot;
    e_theta_dot = theta_d_2 - theta_dot;
    e_psi_dot = psi_d_2 - psi_dot;

    [t_phi, e_phi_dot_int] = PID(e_phi_dot, e_phi_dot_int_prev, K_phi_dot_P,  K_phi_dot_D,  K_phi_dot_D, dt);
    [t_theta, e_theta_dot_int] = PID(e_theta_dot, e_theta_dot_int_prev, K_theta_dot_P,  K_theta_dot_D,  K_theta_dot_D, dt);
    [t_psi, e_psi_dot_int] = PID(e_psi_dot, e_psi_dot_int_prev, K_psi_dot_P,  K_psi_dot_D,  K_psi_dot_D, dt);

    t_B = 1e-6 * [t_phi; t_theta; t_psi];

    e_phi_int_prev = e_phi_int;
    e_theta_int_prev = e_theta_int;
    e_psi_int_prev = e_psi_int;

    e_phi_dot_int_prev = e_phi_dot_int;
    e_theta_dot_int_prev = e_theta_dot_int;
    e_psi_dot_int_prev = e_psi_dot_int;

    %% Drone model
    state = [x; y; z;...
        x_dot; y_dot; z_dot;...
        phi; theta; psi;...
        phi_dot; theta_dot; psi_dot];

    [~, x_ode] = ode45(@(t, x_ode) droneModel(x_ode, g, t_B, T, m, I_xx, I_yy, I_zz), [(i-1) * dt, i * dt], state);

    state = x_ode(max(size(x_ode)), :)'; % drone dynamic integration

    %% Professor wants
    x = state(1); % position X - Ground Truth
    y = state(2); % position Y - Ground Truth
    z = state(3); % position Z - Ground Truth

    x_dot = state(4); % Velocity X - GroundTruth
    y_dot = state(5); % Velocity X - GroundTruth
    z_dot = state(6); % Velocity X - GroundTruth

    phi = state(7); % Euler Angle X (Roll) - GroundTruth
    theta = state(8); % Euler Angle Y (Pitch) - GroundTruth
    psi = state(9); % Euler Angle Z (Yaw) - GroundTruth

    phi_dot = state(10);   % Angular Velocity X (rad/s) - IMU - w_x
    theta_dot = state(11); % Angular Velocity Y (rad/s) - IMU - w_y
    psi_dot = state(12);   % Angular Velocity Z (rad/s) - IMU - w_z

    state_dot = droneModel(state, g, t_B, T, m, I_xx, I_yy, I_zz);
    x_dot_dot = state_dot(4); % Acceleration X (m/s^2) - IMU - a_x
    y_dot_dot = state_dot(5); % Acceleration Y (m/s^2) - IMU - a_y
    z_dot_dot = state_dot(6); % Acceleration Z (m/s^2) - IMU - a_z
    
    %calculate C
    Cbn = euler2dcm([phi; -theta; -psi]);

    % Euler(:, i) = [theta; phi; -psi];  % ENU -> NED
    angular_vel(:, i) = [phi_dot; -theta_dot; -psi_dot] + 0.0087 * ones(3, 1) + 0.0135 * sqrt(100) * pi / 180 * randn(3, 1); % ENU -> NED
    specific_force(:, i) = Cbn' * ([x_dot_dot; -y_dot_dot; -z_dot_dot] - [0;0;g]) + 1.9620 * ones(3, 1) + 0.2256 * randn(3, 1) ; % ENU -> NED

    ground_truth_position(:, i) = [x; y; z]; % ENU
    ground_truth_velocity(:, i) = [x_dot; y_dot; z_dot]; % ENU
    ground_truth_euler(:, i) = [phi; theta; psi];

    %%
    x_plot(:, i) = state;
    x_d_plot(:, i) = [x_d; y_d; z_d; phi_d; theta_d; psi_d];
end

ground_truth_bias_gyro = 0.0087;
ground_truth_bias_accel = 1.9620;
gyro_noise = 0.0135 * sqrt(100) * pi / 180;
accel_noise = 0.2256;
save('../Datasets/drone_IMU.mat', 't', 'angular_vel', 'specific_force', 'ground_truth_position', 'ground_truth_velocity', 'ground_truth_euler','ground_truth_bias_gyro', 'ground_truth_bias_accel', 'gyro_noise', 'accel_noise');

%% Plot
lineWidth = 1.5;

figure(1);
plot3(x_d_plot(1, :), x_d_plot(2, :), x_d_plot(3, :), 'b', 'LineWidth', lineWidth, 'DisplayName', 'Reference');
hold on;
xlim([-1, 1]); ylim([-1, 1]); zlim([0, z_d + 1])
grid on;

annimation = 0;

if annimation == 1
    for i = 1:1:length(x_plot)
        scatter3(x_plot(1, i), x_plot(2, i), x_plot(3, i), 3, 'r', 'filled');
        pause(0.00001);
    end
end

clf
plot3(x_d_plot(1, :), x_d_plot(2, :), x_d_plot(3, :), 'b', 'LineWidth', lineWidth, 'DisplayName', 'Reference');
hold on;
% xlim([-1, 1]); ylim([-1, 1]); zlim([0, z_d + 1])
grid on;
plot3(x_plot(1, :), x_plot(2, :), x_plot(3, :), 'r', 'LineWidth', lineWidth, 'DisplayName', 'PID');
legend;

%%
figure(2);
subplot(311)
plot(x_d_plot(1, :), 'b', 'LineWidth', lineWidth, 'DisplayName', 'Reference');
hold on;
plot(x_plot(1, :), 'r', 'LineWidth', lineWidth, 'DisplayName', 'PID');
grid on; xlim([0 length(x_d_plot)])
legend;

subplot(312)
plot(x_d_plot(2, :), 'b', 'LineWidth', lineWidth);
hold on;
plot(x_plot(2, :), 'r', 'LineWidth', lineWidth);
grid on; xlim([0 length(x_d_plot)])

subplot(313)
plot(x_d_plot(3, :), 'b', 'LineWidth', lineWidth);
hold on;
plot(x_plot(3, :), 'r', 'LineWidth', lineWidth);
grid on; xlim([0 length(x_d_plot)])

%%
figure(3);
subplot(311)
plot(x_d_plot(1, :) - x_plot(1, :), 'r', 'LineWidth', lineWidth, 'DisplayName', 'Error');
grid on; xlim([0 length(x_d_plot)])
legend;

subplot(312)
plot(x_d_plot(2, :) - x_plot(2, :), 'r', 'LineWidth', lineWidth);
grid on; xlim([0 length(x_d_plot)])

subplot(313)
plot(x_d_plot(3, :) - x_plot(3, :), 'r', 'LineWidth', lineWidth);
grid on; xlim([0 length(x_d_plot)])
