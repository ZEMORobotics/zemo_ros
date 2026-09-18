# ZEMO Platform Setup

## `install_script.sh`

This script is **not meant to be executed**.
It serves as a list of all APT packages required by the ZEMO platform.
Whenever a new required package is installed using `apt`, it should also be added to this file.

## `configure_raspberry.sh`

This script configures the Raspberry Pi environment and adds useful ROS 2 commands and settings to `.bashrc`.

For example:
```bash
test -f /opt/ros/jazzy/setup.bash && source /opt/ros/jazzy/setup.bash
test -f "$HOME/ros2_ws/install/local_setup.bash" && source "$HOME/ros2_ws/install/local_setup.bash"
```

This automatically sources:

* ROS 2 Jazzy
* the local `ros2_ws` workspace

whenever a new terminal is opened.

## `cb`

`configure_raspberry.sh` adds the `cb` command to `.bashrc`.
`cb` should be used as the default command for building the ROS 2 workspace.

Internally, it runs:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

After a successful build, it automatically sources:

```bash
install/local_setup.bash
```

Additional `colcon build` arguments can also be passed directly to `cb`.
For example:

```bash
cb --packages-select <package_name>
```

This makes it possible to use normal `colcon build` options while keeping the default ZEMO build configuration.

## temp

```temp``` is a command alias for raspberry temperature monitor. Anywhere you type ```temp``` you will have to enter sudo password.
Temperature will update every second, use ctrl+c to exit.

## SPI to CAN

Setup mcp251xfd can to spi controller:

```
cd /boot/firmware/
```

Then find config.txt and modify it so that it contains:
```
dtparam=spi=on
dtoverlay=mcp251xfd,spi0-0,interrupt=25,oscillator=20000000
```

spi0-0, interrupt and oscillator are depended on hardware configuration of the controller
