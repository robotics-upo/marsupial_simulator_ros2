#!/bin/bash

# Non-interactive mode
export DEBIAN_FRONTEND=noninteractive

# Colors
GREEN='\033[1;32m'
YELLOW='\033[1;33m'
CYAN='\033[1;36m'
RESET='\033[0m'

echo -e "${YELLOW}Starting ROS 2 Humble installation...${RESET}"

# Update system
echo -e "${CYAN}Updating system packages...${RESET}"
apt update -y && apt upgrade -y

# Install ROS 2 Humble
echo -e "${CYAN}Installing ROS 2 Humble...${RESET}"
apt install -y software-properties-common
add-apt-repository -y universe
apt install -y curl
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list
apt update
apt install -y ros-humble-desktop

# Install colcon
echo -e "${CYAN}Installing colcon...${RESET}"
apt install -y python3-colcon-common-extensions

# Install Gazebo and dependencies
echo -e "${CYAN}Installing Gazebo 11.10.2...${RESET}"
apt install -y gazebo=11.10.2+dfsg-1

echo -e "${CYAN}Installing ROS 2 Gazebo packages...${RESET}"
apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
               ros-humble-gazebo-msgs ros-humble-gazebo-plugins ros-humble-gazebo-ros

# Install ros2_control packages (providing ros2 control functionality)
echo -e "${CYAN}Installing ros2_control packages...${RESET}"
apt install -y ros-humble-ros2-control ros-humble-ros2-controllers

# Setup ROS 2 environment
echo -e "${CYAN}Setting up ROS 2 environment...${RESET}"
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash

# Install transforms3d Python module (required by attach_tether.py)
echo -e "${CYAN}Installing transforms3d Python module...${RESET}"
apt install -y python3-pip
pip3 install transforms3d

# Detect the correct src directory where this script is located
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
SRC_DIR="$(dirname "$SCRIPT_DIR")"  # Moves up one level to get the src directory

echo -e "${CYAN}Cloning repositories in: $SRC_DIR ${RESET}"

cd "$SRC_DIR"

# Clone sjtu_drone (branch: ros2)
if [ ! -d "$SRC_DIR/sjtu_drone" ]; then
    echo -e "${CYAN}Cloning sjtu_drone (ros2 branch)...${RESET}"
    git clone -b ros2 --single-branch https://github.com/noshluk2/sjtu_drone.git
else
    echo -e "${YELLOW}sjtu_drone already exists, skipping clone.${RESET}"
fi

# Clone gazebo_ros_link_attacher (branch: humble-devel)
if [ ! -d "$SRC_DIR/gazebo_ros_link_attacher" ]; then
    echo -e "${CYAN}Cloning gazebo_ros_link_attacher (humble-devel branch)...${RESET}"
    git clone -b humble-devel --single-branch https://github.com/davidorchansky/gazebo_ros_link_attacher.git
else
    echo -e "${YELLOW}gazebo_ros_link_attacher already exists, skipping clone.${RESET}"
fi

# Return to workspace root (assuming workspace is one level above src)
cd "$(dirname "$SRC_DIR")"

# Build workspace
echo -e "${CYAN}Building the workspace...${RESET}"
colcon build --symlink-install

# Source workspace
echo -e "${CYAN}Sourcing workspace...${RESET}"
echo "source $(dirname "$SRC_DIR")/install/setup.bash" >> ~/.bashrc
source "$(dirname "$SRC_DIR")/install/setup.bash"

# Apply environment changes
echo -e "${CYAN}Applying environment changes...${RESET}"
source ~/.bashrc
exec bash

# Check ROS 2 installation
if command -v ros2 &> /dev/null
then
    echo -e "${GREEN}ROS 2 is installed and sourced correctly!${RESET}"
else
    echo -e "${RED}ERROR: ROS 2 is not detected! Try running 'source ~/.bashrc' manually.${RESET}"
fi

echo -e "${GREEN}Installation complete.${RESET}"
