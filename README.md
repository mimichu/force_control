# force_control

Implementation of 6D Cartesian space hybrid force-velocity control using positional inner loop and wrist mounted FT sensor.

Author: Yifan Hou
yifanhou at stanford dot edu

# Install
## Dependency
Please install the following packages:
* [cpplibrary](https://github.com/yifan-hou/cpplibrary)

## Build
``` sh
cd force_control
mkdir build && cd build
cmake ..
make
```

# How to use
Refer to `https://github.com/yifan-hou/RobotTestBench` for example usages.

## Reference
The implementation was initially based on
"Experience in Force Control of Robotic Manipulators", James A. Maples and Joseph J. Becker

Then a lot more functionalities were added. Please contact yifan for questions.