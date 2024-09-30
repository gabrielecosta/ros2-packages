# Ros2 Packages

Author: Gabriele Nicolò Costa

Here you will find all my packages created in ROS2. Each package is intented to be stand-alone, but there could be some packages that needs some other packages: I highly recommend you to read *info.md* file in each package before going forward.

Do you have any questions or suggestions? Write me an email!

## Commands to remember
For creating a new package:
```
ros2 pkg create --build-type ament_python --license Apache-2.0 [name_pkg]
```
For building your new package:
```
colcon build --packages-select [name_pkg]
```
For build all packages:
```
colcon build
```
For installing new packages and dependencies:
```
source install/setup.bash
```

## Packages
- **py_camera_stream**: this package allows video streaming over network using udp streaming connections and topic for real time image feed
- **setup**: setup node for camera configuration parameters. It calculates camera calibration matrix and distorsion coefficients and saves them for further applications. It is implemented as service.
- **aruco detector**: aruco detector node for detecting and reporting aruco markers in a scene
- **py object detetion node**: package for testing new YOLOv10 object detection model
- **py joystick**: package for reading input from a joytstick and publishing into a topic for remote controlling
- **py_controller**: this package allows to reading input commands from a controller and translating them into wheels command. It uses a Nucleo Interface library for mapping controller commands and writing them in a specific format in specific registry.

## Collaborations
This repository is in collaboration with Mobius Robotics (University of Palermo).
<div style="text-align: center;">
    <img src="Logo.svg" alt="Mobius Robotics Team" width="200" height="auto"/>
</div>