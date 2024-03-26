function omega = angular2omega(gyro_hat)
w_x_hat = gyro_hat(1, 1);
w_y_hat = gyro_hat(2, 1);
w_z_hat = gyro_hat(3, 1);
omega = [0  -w_x_hat -w_y_hat -w_z_hat;
         w_x_hat  0  w_z_hat  -w_y_hat;
         w_y_hat  -w_z_hat  0  w_x_hat;
         w_z_hat  w_y_hat  -w_x_hat  0];
end