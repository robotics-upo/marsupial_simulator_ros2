FROM osrf/ros:humble-desktop

# Install required system packages and Git LFS:
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
    git-lfs \
    && rm -rf /var/lib/apt/lists/*

# Initialize Git LFS
RUN git lfs install

# Set display environment variable for GUI support.
ENV DISPLAY=:0

# (Opcional) Define GAZEBO_MODEL_PATH para que Gazebo encuentre los modelos
ENV GAZEBO_MODEL_PATH=/home/upo/marsupial/src/marsupial_simulator_ros2/models

# Set the workspace root directory.
WORKDIR /home/upo/marsupial

# Create the src directory inside the workspace.
RUN mkdir -p src

# Set working directory to src and clone each repository from GitHub.
WORKDIR /home/upo/marsupial/src
RUN git clone -b main https://github.com/robotics-upo/marsupial_simulator_ros2.git && \
    git clone -b ros2 https://github.com/noshluk2/sjtu_drone.git && \
    git clone -b humble-devel https://github.com/davidorchansky/gazebo_ros_link_attacher.git && \
    git clone -b humble https://github.com/ros-simulation/gazebo_ros2_control.git && \
    cd marsupial_simulator_ros2 && git lfs pull && cd .. && \
    cd sjtu_drone && git lfs pull && cd .. && \
    cd gazebo_ros_link_attacher && git lfs pull && cd .. && \
    cd gazebo_ros2_control && git lfs pull && cd ..

# Go back to the workspace root.
WORKDIR /home/upo/marsupial

# Initialize rosdep and install dependencies declared in package.xml files.
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
