FROM osrf/ros:humble-desktop

# Install required system packages:
# - build-essential: basic build tools.
# - python3-colcon-common-extensions: build system for ROS2 workspaces.
# - libgazebo-dev: development libraries for Gazebo.
# - python3-rosdep: tool for dependency management in ROS.
# - python3-pip: Python package installer.
# - python3-scipy: scientific computing library.
# - ros-humble-gazebo-ros-pkgs: meta-package for Gazebo ROS integration.
# - ros-humble-xacro: XML macro language for ROS.
# - xterm: terminal emulator.
# - ros-humble-imu-tools: IMU tools for ROS.
# - ros-humble-joint-state-publisher: publishes joint states for robot models.
# - ros-humble-ros2-control: ROS2 control framework.
# - ros-humble-ros2-controllers: common controllers for ROS2 Control.
# - libgl1-mesa-glx, libx11-6: libraries for OpenGL and X11 support.
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    libgazebo-dev \
    python3-rosdep \
    python3-pip \
    python3-scipy \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    xterm \
    ros-humble-imu-tools \
    ros-humble-joint-state-publisher \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    libgl1-mesa-glx \
    libx11-6 \
    && rm -rf /var/lib/apt/lists/*

# Set display environment variable for GUI support.
ENV DISPLAY=:0

# Set the workspace root directory.
WORKDIR /home/upo/marsupial

# Create the src directory inside the workspace.
RUN mkdir -p src

# Set working directory to src and clone each repository from GitHub.
WORKDIR /home/upo/marsupial/src
RUN git clone -b master https://github.com/robotics-upo/marsupial_simulator_ros2.git && \
    git clone -b ros2 https://github.com/noshluk2/sjtu_drone.git && \
    git clone -b humble-devel https://github.com/davidorchansky/gazebo_ros_link_attacher.git && \
    git clone -b humble https://github.com/ros-simulation/gazebo_ros2_control.git

# Go back to the workspace root.
WORKDIR /home/upo/marsupial

# Initialize rosdep and install dependencies declared in package.xml files.
# Removing the default rosdep source file to avoid re-initialization conflicts.
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths /home/upo/marsupial --ignore-src -r -y

# Build the entire ROS2 workspace using colcon from the workspace root.
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Create an entrypoint script to automatically source the workspace environment.
RUN echo '#!/bin/bash\nsource /home/upo/marsupial/install/setup.bash\nexec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

# Set the entrypoint so that the workspace environment is loaded on container start.
ENTRYPOINT ["/entrypoint.sh"]
# Default command: start an interactive bash shell.
CMD ["/bin/bash"]
