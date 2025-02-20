#!/bin/bash

# Non-interactive mode
export DEBIAN_FRONTEND=noninteractive

# Colors
GREEN='\033[1;32m'
YELLOW='\033[1;33m'
CYAN='\033[1;36m'
RESET='\033[0m'

echo -e "${YELLOW}Starting ROS 2 Humble installation...${RESET}"

# Update packages
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

# Setup ROS 2 environment
echo -e "${CYAN}Setting up ROS 2 environment...${RESET}"
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash

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
