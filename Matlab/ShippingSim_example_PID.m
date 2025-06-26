% Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
% Licensed under the MIT License.
% Author: Siemen Herremans

%% PID Example
%
% This example works well with the default example settings found in the docs folder op the repository. 
%
% This example will:
%   -Connect to IDLab-ShippingSim
%   -Get sensor IMU data
%   -Get instance segmented LiDAR Point Cloud
%
% Do note that the AirSim matlab client has almost all API functions available but
% not all are listed in this test script. For a full list see the source code fo the AirSimClient class. 

%% Setup connection

%Define client
vehicle_name = "MilliAmpere1";
airSimClient = AirSimClient(IsDrone=false, IP="127.0.0.1", port=41451);
airSimClient.setEnableApiControl(vehicle_name);

%% Control Loop
desired_speed = 0.75;

% Define PID gains
Kp = 1.5;
Ki = 1.0;
Kd = 0.2;

% Create PID controller
pidController = SimplePID(Kp, Ki, Kd);


while true
    % Get GPS Measurement
    [gnssData, ~, ~] = airSimClient.getGpsData("Gps",'');
    velocity = gnssData.velocity;
    current_speed = sqrt(velocity(1).^2 + velocity(2).^2)

    % Calculate thrust using PID controller
    thrust = pidController.compute(desired_speed, current_speed)
    
    % Set vessel controls with calculated thrust
    airSimClient.setVesselControls(thrust, 0.5, vehicle_name);
    
    % Pause for a short duration to simulate real-time control
    pause(0.1);
end