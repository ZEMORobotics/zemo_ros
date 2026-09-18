# Zemo ROS2 platform
This repository contains all ros2 software for eurobot.

---

# Setup

In *zemo_scripts* directory there is *install_script.sh* that contains all downloaded packages used by this repository. One can view it as a _dependency list_.

- configure_raspberry.bash exports all ros2 variables and helper functions to .bashrc.
- configure_FIFO_permissions.sh configures permissions for ros2 control package.

For further information check README in *zemo_scripts* directory.

# Zemo Hardware

This package is the main package, it contains all communication and hardware drivers that are used on raspberry. Mainly it communicates wheel speeds with motion board.

This package handles:

- Can interface
- Motion board
- IR sensors
- CINCH
- Lidar

TODO: Servo drivers

# Zemo Description

- This packages contains urdf (unique robot description file) for zemo robot. Here all ros2 control connections and robot dimensions are written.

- This file is very important as it containt robot_state_publisher. This publisher exposes all robot transform matricies used by ros2_control, navigation (nav2) and rviz.

- Therefore it is very important that relative dimensions, and frame links are correctly setup in this package

TODO: write description for all packages when they are done.
