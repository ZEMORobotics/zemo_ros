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

## UART setup

In the same *config.txt* found in */boot/firmware/config.txt*, modify it so that it contains:

```
dtoverlay=uart1-pi5
dtoverlay=uart2-pi5
```

This maps pins 27, 28, and 7, 29 to UART interface.

This is as per **andjelko** raspberry shield design

- /dev/ttyAMA0 is uart0 (gpio 14, 15) (TX_EN: 23, RX_EN: 24) WVS uart on shield schematic
- /dev/ttyAMA1 is uart1 (gpio 0, 1) (TX_EN: 6, RX_EN: 7) Dynamixel uart on shield schematic
- /dev/ttyAMA2 is uart2 (gpio 4, 5)
- /dev/ttyAMA10 is debug uart, it should not be used.

You can check exact gpios with:

```bash
sudo cat /sys/kernel/debug/pinctrl/1f000d0000.gpio-pinctrl-rp1/pinmux-pins
```

# DDS setup

For DDS setup two functions are given:

- local_dds : Configure DDS so that it is only on the host machine, this lowers Wi-Fi traffic and improves SIMA communication.

- subnet_dds : This is the default confg. This function configures DDS to LAN so that other devices can see ros2 topics. Used for debbuging.

In ```/etc/systemd/system/wifi-powersave-off.service``` is a daemon service to turn off power_save with ```iw dev wlan0 set power_save off```.
You can check if power_save is on or off with: ```iw wlan0 get power_save```. *wlan0* is network interface, if network is changed and wlan0 does not work, 
you can check the interface with ```iw dev```.


