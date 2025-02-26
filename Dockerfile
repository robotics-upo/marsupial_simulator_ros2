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

# Set the working directory to the ROS2 workspace.
WORKDIR /home/upo/marsupial

# Copy the entire workspace content into the container.
COPY . /home/upo/marsupial

# Clone gazebo_ros2_control from GitHub using the "humble" branch in the src folder.
RUN if [ ! -d "src/gazebo_ros2_control" ]; then \
      mkdir -p src && cd src && \
      git clone -b humble https://github.com/ros-simulation/gazebo_ros2_control.git; \
    fi

# Initialize rosdep and install dependencies declared in package.xml files.
# The default rosdep source file is removed to avoid re-initialization conflicts.
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths /home/upo/marsupial --ignore-src -r -y

# Build the entire ROS2 workspace (including the cloned gazebo_ros2_control package) using colcon.
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Create an entrypoint script to automatically source the workspace environment.
RUN echo '#!/bin/bash\nsource /home/upo/marsupial/install/setup.bash\nexec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

# Set the entrypoint so that the workspace environment is loaded on container start.
ENTRYPOINT ["/entrypoint.sh"]

# Default command: start an interactive bash shell.
CMD ["/bin/bash"]
