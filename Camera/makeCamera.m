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

%%
close all;
clear;
clc;

%%
addpath("../Tools/");
load("../Datasets/drone_IMU.mat");
arglist = {'Marker', 'o', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'LineStyle', 'none'};

f_cam = 20; % camera frequency = 20 Hz
Tcam = 1/f_cam;
Tsim = 30; % simulation time (second) - can be found in drone.m
factor = (length(ground_truth_position)-1)/(Tsim/Tcam);
Ncam = floor(Tsim/Tcam);
d2r = pi/180; r2d = 1/d2r;

debug = 0;
makeGIF = 0; % this require gif from Add-on: https://www.mathworks.com/matlabcentral/fileexchange/63239-gif

%% Add feature
% Create landmarks along the trajectory
left_feature = [];
right_feature = [];
for i = 2:1:length(ground_truth_position)
    if ground_truth_position(3, i) > 0.5
        if ground_truth_position(1, i) < 0 % left plane
            left_feature(:, end+1) = [(ground_truth_position(1, i - 1) - 1) + (ground_truth_position(1, i - 1) + 2 - ground_truth_position(1, i - 1)) * rand(1, 1);...
                                                   (ground_truth_position(2, i - 1) - 1) + (ground_truth_position(2, i - 1) + 2 - ground_truth_position(2, i - 1)) * rand(1, 1);...
                                                   0];
        else
            right_feature(:, end+1) = [(ground_truth_position(1, i - 1) - 1) + (ground_truth_position(1, i - 1) + 2 - ground_truth_position(1, i - 1)) * rand(1, 1);...
                                                      (ground_truth_position(2, i - 1) - 1) + (ground_truth_position(2, i - 1) + 2 - ground_truth_position(2, i - 1)) * rand(1, 1);...
                                                      0];
        end
    end
end

if debug == 1
    figure()
    plot3(ground_truth_position(1, :), ground_truth_position(2, :), ground_truth_position(3, :), 'r', 'LineWidth', 2);
    hold on;
    plot3(left_feature(1, :), left_feature(2, :), left_feature(3, :), arglist{:})
    plot3(right_feature(1, :), right_feature(2, :), right_feature(3, :), arglist{:})
    grid on;
end

% remove some bad feature
left_feature_ = [];
right_feature_ = [];
for i = 1:1:length(left_feature)
    if left_feature(1, i) < -0.2
        left_feature_(:, end+1) = left_feature(:, i);
    end
    if right_feature(1, i) > 0.2
        right_feature_(:, end+1) = right_feature(:, i);
    end
end

if debug == 1
    figure()
    plot3(ground_truth_position(1, :), ground_truth_position(2, :), ground_truth_position(3, :), 'r', 'LineWidth', 2);
    hold on;
    plot3(left_feature_(1, :), left_feature_(2, :), left_feature_(3, :), arglist{:})
    plot3(right_feature_(1, :), right_feature_(2, :), right_feature_(3, :), arglist{:})
    xlabel X
    ylabel Y
    zlabel Z
    grid on;
end

% increase distance = reduce number of points
left_distance = 0.3;
left_feature__ = removeCloseFeature(left_feature_, left_distance);

right_distance = 0.3;
right_feature__ = removeCloseFeature(right_feature_, right_distance);

if debug == 1
    figure()
    plot3(ground_truth_position(1, :), ground_truth_position(2, :), ground_truth_position(3, :), 'r', 'LineWidth', 2);
    hold on;
    plot3(left_feature__(1, :), left_feature__(2, :), left_feature__(3, :), arglist{:})
    plot3(right_feature__(1, :), right_feature__(2, :), right_feature__(3, :), arglist{:})
    grid on;
end

feature3D_G = [left_feature__, right_feature__];
% minPlotX = min(feature3D_G(1,:));
maxPlotX = max(max(feature3D_G(1, :)), abs(min(feature3D_G(1, :))));

% minPlotY = min(feature3D_G(2,:));
maxPlotY = max(max(feature3D_G(2, :)), abs(min(feature3D_G(2, :))));

%% Making camera image
% Camera model
pixel = 10e-6;
f = 0.0128; % smaller f gives a bigger Field of view
f_pixel = f / pixel; % focal length in the unit of pixel

%setup image
imageSize = [1280, 1024];

% Principal point
u0 = imageSize(1) / 2;
v0 = imageSize(2) / 2;

% setup camera with focal length f, centre u0,v0
cameraMatrix = [  f_pixel,  0  ,  u0;
                           0,  f_pixel   ,  v0;
                           0,  0  ,  1];
distortion = [0, 0, 0, 0, 0];

len =0.5;
WX = [len; 0; 0]; WY = [0; len; 0]; WZ = [0; 0; len];

% IMU - Camera Transformation
I_R_C = euler2dcm([0; pi; pi/2]);
I_position_C = [0;0;0];
n = 0;
feature2D_record = zeros(2, size(feature3D_G,2), Ncam);
cameraHomoMatrix = zeros(4, 4, Ncam);

fg = figure; 
set(fg,'units','normalized','position',[0.2 0.2 0.6 0.6]);

for i = 1:1:length(ground_truth_position)
    % create a camera transformation matrix The transformation that places
    % the camera in the correct position and orientation in world space
    % Camera Rotation matrix with respect to Global
    G_R_I = euler2dcm(ground_truth_euler(:, i));
%     imuTrans = -G_R_I' * ground_truth_position(:, i);

    if mod(i, factor) == 0
        n = floor(i  / factor);
         
        %  facedDownCoor =   G_R_I * C_R_I'; % To keep the camera facing downward + change with the drone rotation C_C^G
        facedDownCoor = G_R_I * I_R_C;
        camTrans = -facedDownCoor' * ground_truth_position(:, i);
               
        % Camera pose in Global frame
        cameraHomoMatrix(1:3, 1:3,n) =  facedDownCoor';
        cameraHomoMatrix(1:3, 4, n) = -facedDownCoor' * (ground_truth_position(:, i) + G_R_I * I_position_C);
        
        %--------------- Calculation ------------------
%         C_ = ground_truth_position(:, i);
%         R_ = facedDownCoor';

        [projected, valid] = projectPoints(feature3D_G', cameraMatrix, cameraHomoMatrix(:, :, n), distortion, imageSize);
        feature2D = projected';

        % ---------------Save data-------------------
        for k = 1:1:length(valid)
            if valid(k) == 1
                feature2D_record(:, k, n) = feature2D(:, k);
            else
                feature2D_record(:, k, n) = NaN;
            end
        end
        % ---------------------------------------------
        
        % Calculate plot variable ( the green box )
%         visualPlot = G_R_I' * ground_truth_position(:, i);
        visualPlot = ground_truth_position(:, i);
        visualPlotCentral = repmat(ground_truth_position(:, i),1,4);
%         viet_x = imageSize(1) * f;
%         viet_y = imageSize(2) * f;
        viet_x = 1.7/2;
        viet_y = 1.1/2;
        recPlot = [visualPlot(1) - viet_x, visualPlot(1) + viet_x, visualPlot(1) + viet_x, visualPlot(1) - viet_x, visualPlot(1) - viet_x; ...
                        visualPlot(2) - viet_y, visualPlot(2) - viet_y, visualPlot(2) + viet_y, visualPlot(2) + viet_y, visualPlot(2) - viet_y; ...
                        0, 0, 0, 0, 0 ];

%         for ii = 1:1:length(recPlot)
%             recPlot(:, ii) = G_R_I * recPlot;
%         end
        
        %--------------- Plot 3D------------------
%         figure(100),
%         set(gcf,'WindowState','maximized')
%         set(gca, 'zdir', 'reverse')

        clf;
        subplot(1,2,1)
        hold on;grid on; box on;
        xlim([-maxPlotX-1 maxPlotX+1]);
        ylim([-maxPlotY-1 maxPlotY+1]);
        zlim([0 2.5]);
        
%         World coordinate frame
%         plot3([0 WX(1)],[ 0 WX(2)],[0 WX(3)],'b','LineWidth',2);text(WX(1), WX(2), WX(3), 'X_{w}'); 
%         plot3([0 WY(1)],[0 WY(2)],[0 WY(3)],'g','LineWidth',2);text(WY(1), WY(2), WY(3), 'Y_{w}');
%         plot3([0 WZ(1)],[0 WZ(2)],[0 WZ(3)],'r','LineWidth',2);text(WZ(1), WZ(2), WZ(3), 'Z_{w}');
%         text(0,0,0,'O_{w}');
%         ------------------------------------------------------------
        
%         Plot IMU coordinate frame
%         imuCoor_ = ground_truth_position(:, i);
%         imuCoorX = G_R_I * (WX - imuTrans);
%         imuCoorY = G_R_I * (WY - imuTrans);
%         imuCoorZ = G_R_I * (WZ - imuTrans);
% 
%         plot3([imuCoor_(1) imuCoorX(1)],[imuCoor_(2) imuCoorX(2)],[imuCoor_(3) imuCoorX(3)],'b','LineWidth',2);text(imuCoorX(1), imuCoorX(2), imuCoorX(3), 'X_{I}');
%         plot3([imuCoor_(1) imuCoorY(1)],[imuCoor_(2) imuCoorY(2)],[imuCoor_(3) imuCoorY(3)],'g','LineWidth',2);text(imuCoorY(1), imuCoorY(2), imuCoorY(3), 'Y_{I}');
%         plot3([imuCoor_(1) imuCoorZ(1)],[imuCoor_(2) imuCoorZ(2)],[imuCoor_(3) imuCoorZ(3)],'r','LineWidth',2);text(imuCoorZ(1), imuCoorZ(2), imuCoorZ(3), 'Z_{I}');
%         text(imuCoor_(1), imuCoor_(2), imuCoor_(3), 'O_{I}');
%         ------------------------------------------------------------
        
%         Plot trajectory
        plot3(ground_truth_position(1, :), ground_truth_position(2, :), ground_truth_position(3, :), 'r', 'LineWidth', 2);
        hold on;

%         Plot Camera coordinate frame
        camCoor_ = ground_truth_position(:, i);
        camCoorX = facedDownCoor * (WX - camTrans);
        camCoorY = facedDownCoor * (WY - camTrans);
        camCoorZ = facedDownCoor * (WZ - camTrans);
        plot3([camCoor_(1) camCoorX(1)],[camCoor_(2) camCoorX(2)],[camCoor_(3) camCoorX(3)],'b','LineWidth',2);
        plot3([camCoor_(1) camCoorY(1)],[camCoor_(2) camCoorY(2)],[camCoor_(3) camCoorY(3)],'g','LineWidth',2);
        plot3([camCoor_(1) camCoorZ(1)],[camCoor_(2) camCoorZ(2)],[camCoor_(3) camCoorZ(3)],'r','LineWidth',2);
%         text(camCoorX(1), camCoorX(2), camCoorX(3), 'X_{C}');
%         text(camCoorY(1), camCoorY(2), camCoorY(3), 'Y_{C}');
%         text(camCoorZ(1), camCoorZ(2), camCoorZ(3), 'Z_{C}');
%         text(camCoor_(1), camCoor_(2), camCoor_(3), 'O_{C}');
        
%         Plot 3D landmarks
        plot3(feature3D_G(1, :), feature3D_G(2, :), feature3D_G(3, :), arglist{:});
%         ------------------------------------------------------------
        
%         Plot drone's camera visualization
        plot3(recPlot(1, :), recPlot(2, :), recPlot(3, :), 'g', 'LineWidth', 2);
        plot3([visualPlotCentral(1, :); recPlot(1, 1:4)], [visualPlotCentral(2, :); recPlot(2, 1:4)], [visualPlotCentral(3, :); recPlot(3, 1:4)], '--b', 'LineWidth', 1);
%         ------------------------------------------------------------
        
        view([20 30]);
        
%         for j=1:size(feature3D_G, 2)
%             text(feature3D_G(1, j), feature3D_G(2, j), sprintf('  %d', j), ...
%                 'HorizontalAlignment', 'left', ...
%                 'VerticalAlignment', 'middle', ...
%                 'FontUnits', 'pixels', ...
%                 'FontSize', 12, ...
%                 'Color', 'k')
%         end

        % Setting up the plot paramater
%         xlim([-2 6]);
%         ylim([-6 2]);
        xlabel('X [m]');
        ylabel('Y [m]');
        zlabel('Z [m]');
%         legend('Trajectory','Landmarks');
        hold off;
        
%         ------------Plot 2D---------------
%         figure(2),
%         clf('reset');
        subplot(1,2,2);
        plot(feature2D(2, :), feature2D(1, :),arglist{:});
%         for j = 1:size(feature2D,2)
%             text(feature2D(2, j), feature2D(1, j), sprintf('  %d', j), ...
%                 'HorizontalAlignment', 'left', ...
%                 'VerticalAlignment', 'middle', ...
%                 'FontUnits', 'pixels', ...
%                 'FontSize', 12, ...
%                 'Color', 'k')
%         end
        hold on;
        rectangle('Position',[0 0 imageSize(1) imageSize(2)])
        grid on;
%         set(gca, 'ydir', 'reverse')
        title('Image Plane');
        xlim([0 imageSize(1)]);
        ylim([0 imageSize(2)]);
        xlabel('u [pixel]');
        ylabel('v [pixel]');
        
        pause(1e-5);
    
%         if makeGIF == 1
%             if i == 5
%                 gif('drone_vins.gif', 'DelayTime',1/24)
%             else
%                 gif
%             end
%         end
%         
    end    % Test Break Point
end

%% Save dataset
gT.Time = t;
gT.Position = ground_truth_position;
gT.Velocity = ground_truth_velocity;
gT.Euler = ground_truth_euler;
gT.BiasGyro = ground_truth_bias_gyro * ones(3, length(gT.Position));
gT.BiasAccel = ground_truth_bias_accel * ones(3, length(gT.Position));

IMU.Accel = specific_force;
IMU.Gyro = angular_vel;
IMU.Hz = 100;
IMU.Noise.Accel = accel_noise;
IMU.Noise.Gyro = gyro_noise;

Camera.FeatureMeasurement = feature2D_record;
Camera.Hz = f_cam;
Camera.Intrinsic = cameraMatrix;
Camera.Extrinsic = [euler2dcm([0; 0; pi/2]), zeros(3, 1); 0, 0, 0, 1];
Camera.Distortion = [0, 0, 0, 0];
Camera.Pixel = pixel;
Camera.FocalLength = f;
Camera.P_f_G = feature3D_G;
save('../Datasets/drone_IMU_camera.mat', 'gT', 'IMU', 'Camera');

