# ASVSim - AirSim for Surface Vehicles
![image](https://github.com/user-attachments/assets/9835edca-2f57-4dec-90bd-14bdaa56f2d5)

[Documentation](https://bavolesy.github.io/idlab-asvsim-docs/)
NOTE: We are currently working on this documentation. You may find some incomplete or outdated information. We will update it as soon as possible.

IDLab-ASVSim (AirSim for Surface Vehicles) is a fork of the Cosys-AirSim project, which is a simulator for drones, cars and more, with extensive API support, built on [Unreal Engine](https://www.unrealengine.com/). IDLab-VesSim extends upon Cosys-AirSim by adding simulation of vessels. It is open-source, cross platform, and supports hardware-in-loop with popular flight controllers such as PX4 for physically and visually realistic simulations. It is developed as an Unreal plugin that can simply be dropped into any Unreal environment.

IDLab, (University of Antwerp - imec) created the ASVSim project to simulate autonomous vessels in a realistic environment with the goal of developing and testing autonomous vessel path planning algorithms. The simulator uses Unreal Engine (5.4.X) to create a realistic environment and simulate the vessel's movement. The simulator is designed to be modular and extensible, allowing for easy integration of new features and sensors. Please contact the IDLab researchers to get more in depth information on our work or if you wish to collaborate. As IDLab focuses on autonomous vessels, the focus of this fork is on simulating vessels and their sensors. If you are interested in simulating drones, cars or other vehicles and their sensors, please refer to the [Cosys-AirSim project](https://github.com/Cosys-Lab/Cosys-AirSim).



This fork is based on last public AirSim release from Microsoft's GitHub.
The [original AirSim MIT license](//add license) applies to all native AirSim source files. 
Please note that we use that same [MIT license](//add license) as which applies to all changes made by IDLab in case you plan to do anything within this repository.
Do note that this repository is provided as is, will not be actively updated and comes without warranty or support. 
Please contact a IDLab/Cosys-Lab researcher to get more in depth information on which branch or version is best for your work.

## Associated publications

- [Cosys-AirSim: A Real-Time Simulation Framework Expanded for Complex Industrial Applications](https://arxiv.org/abs/2303.13381)
```
@inproceedings{cosysairsim2023jansen,
  author={Jansen, Wouter and Verreycken, Erik and Schenck, Anthony and Blanquart, Jean-Edouard and Verhulst, Connor and Huebel, Nico and Steckel, Jan},
  booktitle={2023 Annual Modeling and Simulation Conference (ANNSIM)}, 
  title={COSYS-AIRSIM: A Real-Time Simulation Framework Expanded for Complex Industrial Applications}, 
  year={2023},
  volume={},
  number={},
  pages={37-48},
  doi={}}
```

You can also find the presentation of the live tutorial of Cosys-AirSim at ANNSIM '23 conference [here](https://github.com/Cosys-Lab/Cosys-AirSim/tree/main/docs/annsim23_tutorial) together with the associated videos.

- [Physical LiDAR Simulation in Real-Time Engine](https://arxiv.org/abs/2208.10295)
```
@inproceedings{lidarsim2022jansen,
  author={Jansen, Wouter and Huebel, Nico and Steckel, Jan},
  booktitle={2022 IEEE Sensors}, 
  title={Physical LiDAR Simulation in Real-Time Engine}, 
  year={2022},
  volume={},
  number={},
  pages={1-4},
  doi={10.1109/SENSORS52175.2022.9967197}}
}
```
- [Simulation of Pulse-Echo Radar for Vehicle Control and SLAM](https://www.mdpi.com/1424-8220/21/2/523)
```
@Article{echosim2021schouten,
  author={Schouten, Girmi and Jansen, Wouter and Steckel, Jan},
  title={Simulation of Pulse-Echo Radar for Vehicle Control and SLAM},
  JOURNAL={Sensors},
  volume={21},
  year={2021},
  number={2},
  article-number={523},
  doi={10.3390/s21020523}
}
```

## IDLab Modifications
* Added support for vessels ([Note that vessel simulation is the main focus of this fork](link))
* Added example environments and models for vessel simulation
* Added documentation on how to use the simulator for vessel simulation
* Added example code for training a vessel path planning algorithm using reinforcement learning in the simulator
* Added an example of a tuned Dynamic Window Approach (DWA) path planning algorithm for vessels
* Added support for joystick and keyboard control of vessels
* Added MacOS support.
## Cosys-Lab Modifications
* Added support for Unreal up to 5.4 ([Note that Unreal 5.3/5.4 breaks camera scene rendering by default in custom environments](https://cosys-lab.github.io/unreal_custenv#unreal-5354-scene-camera-bug))
* Added [multi-layer annotation](https://cosys-lab.github.io/annotation) for groundtruth label generation with RGB, greyscale and texture options. Extensive API integration and available for camera and GPU-LiDAR sensors.
* Added [Instance Segmentation](https://cosys-lab.github.io/instance_segmentation). 
* Added [Echo sensor type](https://cosys-lab.github.io/echo) for simulation of sensors like sonar and radar.
* Added [GPU LIDAR sensor type](https://cosys-lab.github.io/gpulidar): Uses GPU acceleration to simulate a LiDAR sensor. Can support much higher point density then normal LiDAR and behaves more authentic and has realistic intensity generation.
* Added [skid steering SimMode and vehicle type](https://cosys-lab.github.io/skid_steer_vehicle). ClearPath Husky and Pioneer P3DX implemented as vehicle types using this new vehicle model. 
* Added [Matlab API Client](https://cosys-lab.github.io/matlab) implementation as an easy to install Matlab toolbox.
* Added various [random but deterministic dynamic object types and world configuration options](https://cosys-lab.github.io/dynamic_objects).
* Added BoxCar vehicle model to the Car SimMode to have a smaller vehicle to use in indoor spaces.
* Updated [ComputerVision mode](https://cosys-lab.github.io/image_apis#computer-vision-mode-1): Now has full API and Simulation just like other vehicle types. It mostly means it can now have sensors attached (outside of IMU). Improved handling and camera operation.
* Updated [LIDAR sensor type](https://cosys-lab.github.io/lidar): Fixed not tracing correctly, added ground truth (point labels) generation, added range-noise generation. Improved API pointcloud delivery to be full scan instead of being frame-rate dependent and partial.
* Updated the camera, Echo and (GPU-)LiDAR sensors to be uncoupled from the vehicle and be placed as external world sensors.
* Updated sensors like cameras, Echo sensor and GPU-LiDAR to ignore certain objects with the _MarkedIgnore_ Unreal tag and enabling the "IgnoreMarked" setting in [the settings file](https://cosys-lab.github.io/settings).
* Updated cameras sensor with more distortion features such as chromatic aberration, motion blur and lens distortion. 
* Updated Python [ROS implementation](https://cosys-lab.github.io/ros) with completely new implementation and feature set.
* Updated C++ [ROS2 implementation](https://cosys-lab.github.io/ros) to support custom Cosys-AirSim features.
* Dropped support for Unity Environments.

Some more details on our changes can be found in the [changelog](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/CHANGELOG.md).

## How to Get It
#### Download and run from packaged binary - Windows/Linux
* [Download and run it](https://cosys-lab.github.io/run_packaged)
#### Download and install from precompiled plugin - Windows/Linux
* [Download and install it](https://cosys-lab.github.io/install_precompiled)
#### Install and use from source - Windows
* [Install/Build it](https://cosys-lab.github.io/install_windows)
#### Install and use from source - Linux
* [Install/Build it](https://cosys-lab.github.io/install_linux)
#### Install and use from source - MacOS
* [Install/Build it](docs/install_mac.md)

## How to Use It

### Documentation

View our [detailed documentation](https://cosys-lab.github.io/) on all aspects of Cosys-AirSim.

## Original AirSim Paper

More technical details are available in [AirSim paper (FSR 2017 Conference)](https://arxiv.org/abs/1705.05065). Please cite this as:
```
@inproceedings{airsim2017fsr,
  author = {Shital Shah and Debadeepta Dey and Chris Lovett and Ashish Kapoor},
  title = {AirSim: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles},
  year = {2017},
  booktitle = {Field and Service Robotics},
  eprint = {arXiv:1705.05065},
  url = {https://arxiv.org/abs/1705.05065}
}
```

## License

This project is released under the MIT License. Please review the [License file](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/LICENSE) for more details.
