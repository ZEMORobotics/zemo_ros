# The purpose of this script is to have a list of all downloaded packages.
# If this script is ever used, it is doomsday.

set -e

ROS_DISTRO="jazzy"
sudo usermod -a -G dialout $USER
# Log out and log back in for changes to take effect

# before installing ros2, check for UTF-8 with: locale

# Creating ros2 apt repo ================
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
# =======================================

sudo apt update
sudo apt upgrade

sudo apt-get install build-essential cmake git

sudo apt -y install \
	ros-${ROS_DISTRO}-ros-base \
	ros-dev-tools \
	python3-rosdep \
	ros-${ROS_DISTRO}-xacro \
	ros-${ROS_DISTRO}-ros2-control \
	ros-${ROS_DISTRO}-ros2-controllers \
	ros-${ROS_DISTRO}-rplidar-ros
	can-utils \
	python3-pip \
	htop \
	libgpiod-dev \
	gpiod

