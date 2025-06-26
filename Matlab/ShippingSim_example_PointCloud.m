% Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
% Licensed under the MIT License.
% Author: Siemen Herremans

%% PointCloud Example
%
% This example works well with the default example settings found in the docs folder op the repository. 
%
% This example will:
%   -Connect to IDLab-ShippingSim
%   -Get sensor Gps
%   -Provide a simple PID control example
%
% Do note that the AirSim matlab client has almost all API functions available but
% not all are listed in this test script. For a full list see the source code fo the AirSimClient class. 

%% Setup connection

%Define client
vehicle_name = "MilliAmpere1";
airSimClient = AirSimClient(IsDrone=false, IP="127.0.0.1", port=41451);

%% Groundtruth labels
% Get groundtruth look-up-table of all objects and their instance
% segmentation colors for the cameras and LiDAR
groundtruthLUT = airSimClient.getInstanceSegmentationLUT();

%% IMU sensor Data
imuSensorName = "Imu";
[imuData, imuTimestamp] = airSimClient.getIMUData(imuSensorName, vehicle_name)

%% LiDAR sensor data
% Example plots lidar pointcloud and getting the groundtruth labels

lidarSensorName = "lidar1";
enableLabels = true;
[lidarPointCloud, lidarLabels, LidarTimestamp, LidarSensorPose] = airSimClient.getLidarData(lidarSensorName, enableLabels, vehicle_name);
figure;

rgb = zeros(length(lidarLabels), 3, "uint8");
for i = 1:length(groundtruthLUT.name)
    label = groundtruthLUT(i, :);
    indices = find(lidarLabels == label.name);
    rgb(indices, 1) = label.r;
    rgb(indices, 2) = label.g;
    rgb(indices, 3) = label.b;
end

if ~isempty(lidarPointCloud)
    lidarPointCloud.Color = rgb;
    pcshow(lidarPointCloud, MarkerSize=50);
else
    pcshow(pointCloud([0, 0, 0]));
end
title('Instance Segmented LiDAR Pointcloud')
xlabel("X (m)")
ylabel("Y (m)")
zlabel("Z (m)")
xlim([0 100])
ylim([-100 100])
zlim([-10 10])
drawnow