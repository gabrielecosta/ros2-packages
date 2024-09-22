# Ros2 Packages

Author: Gabriele Nicolò Costa

Here you will find all my packages created in ROS2. Each package is intented to be stand-alone, but there could be some packages that needs some other packages: I recommend you to read *info.md* file in each package before going forward.

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
