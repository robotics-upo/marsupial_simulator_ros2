<div align="center">

  <a href="https://github.com/robotics-upo/marsupial-simulator-ros2">
    <img src="images/logo.png" alt="Logo" width="170" height="100">
  </a>

  <h3 align="center">
    <a href="https://arxiv.org/abs/2412.12776" style="text-decoration: none; color: inherit;">
      Physical simulation of Marsupial UAV-UGV Systems Connected by a Hanging Tether using Gazebo
    </a>
  </h3>
</div>

<div align="center">
  José E. Maese, Fernando Caballero, and Luis Merino  
</div>

<br>

<div align="center">
  <a href="https://youtu.be/ZLLDROIaHV0" 
     style="display: inline-block; margin-right: 10px; text-decoration: none; vertical-align: middle;">
    <img src="https://img.shields.io/badge/YouTube-Video-red?logo=youtube" alt="YouTube Video">
  </a>
  <a href="https://arxiv.org/abs/2412.12776" 
     style="display: inline-block; text-decoration: none; vertical-align: middle;">
    <img src="https://img.shields.io/badge/arXiv-Paper-blue?logo=arxiv" alt="arXiv Paper">
  </a>
</div>


## Table of Contents
<details>

1. [Introduction](#introduction)
2. [System Overview](#system-overview)
   - [Architecture](#architecture)
   - [Models](#models)
3. [Installation](#installation)
   - [Dependencies](#dependencies)
   - [Build Instructions](#build-instructions)
4. [Usage](#usage)
   - [Manual Control](#manual-control)
   - [Automatic Control](#automatic-control)
   - [Experiments](#experiments)
5. [Customization](#customization)
   - [How to modify tether model](#how-to-modify-tether-model)
   - [How to include new scenarios](#how-to-include-new-scenarios)
6. [Cite this work](#Cite-this-work)

</details>

## Introduction
This project presents a ROS 2-based simulator framework for tethered UAV-UGV marsupial systems in Gazebo. The framework models interactions among a UAV, a UGV, and a winch with dynamically adjustable length and slack of the tether. It supports both manual control and automated trajectory tracking, with the winch adjusting the length of the tether based on the relative distance between the robots. The simulator's performance is demonstrated through experiments, including comparisons with real-world data, showcasing its capability to simulate tethered robotic systems. The framework offers a flexible tool for researchers exploring tethered robot dynamics.

<div align="center">
  <img src="images/real_test_gif_2.gif" alt="stage_1 simulation" width="900">
</div>

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## System Overview

### Architecture
The architecture of the marsupial UAV-UGV simulator is composed of key components that interact to replicate the behavior of the tethered robotic system. Built using ROS 2 and Gazebo, the simulator supports both manual and autonomous operations.

- **Model Initialization**: The UGV, UAV, and tether are spawned in Gazebo, with the UAV placed on a platform atop the UGV and the tether initialized in a coiled configuration around the winch.

- **Trajectory Tracking**: A flexible tracking module accepts waypoints (YAML files) or dynamic ROS messages to guide the UAV and UGV. The winch adjusts the tether length in real time based on relative positions to maintain proper slack.

- **Controllers**: Independent controllers manage the UAV and UGV movements, enabling customizable dynamics and the integration of new control strategies.

- **Evaluation and Data Recording**: An evaluation module logs key metrics, such as ground-truth poses, tether behavior, and trajectory accuracy, facilitating performance analysis and validation.

<div align="center">
  <img src="images/simulator_structure_v2.png" alt="Architecture Diagram" width="900">
</div>

This modular architecture allows researchers to customize and test various algorithms and components within a controlled simulation environment. For a detailed explanation, refer to the accompanying paper.



### Models
- **UAV**: Quadrotor with ROS2-compatible position and velocity control.
- **UGV**: Holonomic ground vehicle with integrated winch.
- **Tether**: Flexible, multi-segmented tether with dynamic length adjustment. Configurable length, mass, and stiffness.

Default parameters (e.g., spring stiffness, damping) can be modified as described [here](#modifying-the-tether).

<div align="center">
  <img src="images/marsupial_models.png" alt="Architecture Diagram" width="400">
</div>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Installation

### Dependencies

This package has been designed and tested in an x86_64 machine under a Ubuntu 22.04 operating system and ROS2 Humble distribution. The following repositories are required for the implementation of the project:
  - sjtu_drone: (https://github.com/noshluk2/sjtu_drone/tree/ros2, branch: ros2)
  - gazebo_ros_link_attacher: (https://github.com/davidorchansky/gazebo_ros_link_attacher, branch: humble-devel)

### Build Instructions


1. Clone this repository into the `src` directory of your `colcon` workspace. 

2. clones the required dependencies into the `src` directory.

3. Finally compile your workspace using ```colcon build``` 

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Usage
Six scenarios with different features can be set to use the optimizer. S1: Open environment, S2: Narrow/constrained environment, S3: Confined environment, S5: Open environment, S6: Confined environment, S7: Open environment, as shown in the next figure.

<div align="center">
  <img src="images/all_scenaries.png" alt="scenaries simulation" width="900">
</div>

An extra scenario has been included to replicate the trajectory tracking experiment in a theatre conducted for the Path and Trajectory paper. Uncomment the ```spawn theatre``` line in the ```marsupial_simulation.launch.py``` file to see the theatre model (This can noticeably slow down the simulation on older hardware.). 

<div align="center">
  <img src="images/theatre.png" alt="theatre simulation" width="900">
</div>

The package has a set of predefined configurations (and completely extendable according to the user's need) that relate to the stage number and initial position number. 


### Manual control
To launch the marsupial system in manual mode just launch the file `launch/marsupial_manual_simulation.launch.py`. The control of the ugv can be done using a remote control (default option) or the keyboard. The uav is controlled by the teleop option. To manage the scenario and initial position predefined is recommended to use the parameters for this launch, `world` and `pos_x`, `pos_y`, `pos_z`. Thus, for example, to use S5 and initial position (3, 1, 0):
1. Launch of the gazebo environment:
    ```bash
    ros2 launch marsupial_simulator_ros2 marsupial_manual_simulation.launch.py world:=stage_5.world pos_x:=3 pos_y:=1 pos_z:=0
    ```
2. In order to control the drone it is necessary to send a message for take-off:
    ```bash
    ros2 topic pub /sjtu_drone/takeoff std_msgs/msg/Empty {} --once
    ```
3. Landing message:
    ```bash
    ros2 topic pub /sjtu_drone/land std_msgs/msg/Empty {} --once
    ```

### Automatic control
To launch the marsupial system in automatic mode just launch the file `launch/marsupial_simulation.launch.py`. To manage the scenario and initial position predefined is recommended to use the parameters for this launch, `world` and `pos_x`, `pos_y`, `pos_z`. Thus, for example, to use S5 and initial position (3, 1, 0):
1. Launch of the gazebo environment:
    ```bash
    ros2 launch marsupial_simulator_ros2 marsupial_simulation.launch.py world:=stage_5.world pos_x:=3 pos_y:=1 pos_z:=0
    ```
2. To start the movement to the defined point:
    ```bash
    ros2 launch marsupial_simulator_ros2 marsupial_to_point.launch.py uav_x:=1.0 uav_y:=2.0 uav_z:=7.0 ugv_x:=5.0 ugv_y:=3.0
    ```
3. To change the destination point during simulation:
    - UGV
      ```bash
      ros2 topic pub /target_position_ugv geometry_msgs/msg/Pose '{position: {x: 3.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}' --once
      ```
    - UAV
      ```bash
      ros2 topic pub /target_position_uav geometry_msgs/msg/Pose '{position: {x: 3.0, y: 2.0, z: 7.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}' --once
      ```
4. To record a bag:
    ```bash
    ros2 bag record /sjtu_drone/gt_pose /sjtu_drone/cmd_vel /ugv_gt_pose /forward_velocity_controller/commands /cable_length /target_position_uav /target_position_ugv /tether_positions
    ```


### Experiments
To replicate the experiments conducted just launch the file `launch/marsupial_simulation.launch.py` and `launch/marsupial_experiment.launch.py`. It is recommended to perform the experiments in the predefined scenario to increase the efficiency of the simulator.

1. Launch of the gazebo environment:
    ```bash
    ros2 launch marsupial_simulator_ros2 marsupial_simulation.launch.py
    ```
2. To start the experiment:
    ```bash
    ros2 launch marsupial_simulator_ros2 marsupial_experiment.launch.py mission:=test1
    ```
3. To record a bag: the bag is recorded automatically

The tests carried out are as follows:

- Test 1. The UGV is kept static. UAV ascends and descends N times.
- Test 2. The UAV is kept static. The UGV moves between two points N times.
- Test 3. UAV and UGV move in opposite directions N times.
- Test 4. The test performed inside a theatre mentioned in "Path and Trajectory Planning of a Tethered UAV-UGV Marsupial Robotic System" (https://ieeexplore.ieee.org/document/10207830) is replicated. The length of the tether is calculated as a function of the relative distance between the UAV and the UGV. The test is maintained up to the target point 100.
- Test 5. The test performed inside a theatre is replicated again. In this case, the length of the tether is provided by the test trajectory. The test is maintained up to the target point 100. 

<div align="center">
  <img src="images/tests_examples_v3.png" alt="theatre simulation" width="795">
</div>

It is possible to modify the `self.tether_coef` value of the `ugv_theter_trajectory_follower.py` script to adjust the behaviour of the tether. The test0 is included for this purpose. This parameter should be changed between 0 and 1 until a realistic result is achieved. 

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- Customization -->
## Customization

### How to modify tether model

By adjusting these parameters, you can simulate different types of tethers and study their behavior under various conditions. The parameters of the tether connecting the UAV and UGV can be customized through a Jinja file. This file allows you to adjust various physical properties of the tether elements to better suit your simulation needs. 

1. Open the Jinja file located at `models > tether > tether.sdf.jinja`.

2. Adjust the values of the parameters as needed. To increase the size of the tether, it is recommended to focus on increasing the `number_elements` parameter. This will result in a longer tether composed of more segments.

3. Save the changes to the Jinja file.

4. Recompile your workspace if necessary to apply the changes to the simulation.

5. To implement the changes run:
    ```bash
    python3  ~/marsupial/src/marsupial_simulator_ros2/scripts/jinja_gen.py   ~/marsupial/src/marsupial_simulator_ros2/models/tether/tether.sdf.jinja ~/marsupial/src/marsupial_simulator_ros2/models/tether 
    ```

### How to include new scenarios

You can easily introduce additional scenarios in the simulator by creating or copying a Gazebo `.world` file and placing it in the appropriate directory. This allows you to customize the environment layout (e.g., obstacles, buildings) while preserving the marsupial UAV-UGV simulation functionalities. Follow these steps:

1. Copy the `.world` file into the `worlds` directory
    ```
    ~/marsupial/src/marsupial_simulator_ros2/worlds
    ```
    
2. Add required plugins
    In your .world file, make sure to include the essential plugins for octomap construction and link attachment, as shown below. Insert these lines inside the <world> tag:
    ```xml
    <world name="new:_world">

    <!-- World elements  -->

    <plugin name="gazebo_octomap" filename="libBuildOctomapPlugin.so"/>
    <plugin name="ros_link_attacher_plugin" filename="libgazebo_ros_link_attacher_plugin.so"/>

    </world>
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Cite this work

This simulator has been submitted to a journal and is currently under review. You can see the details in the following arXiv repository.

> J. E. Maese, F. Caballero, and L. Merino. "Physical simulation of Marsupial UAV-UGV Systems Connected by a Hanging Tether using Gazebo", available on [arXiv:2412.12776](https://arxiv.org/abs/2412.12776).

<br>

<sub>This work was partially supported by the INSERTION PID2021-127648OB-C31 and NORDIC TED2021-132476B-I00 projects, funded by MCIN/AEI/10.13039/501100011033 and the European Union NextGenerationEU/PRTR.</sub>

<div align="center">
  <img src="images/fondos.png" alt="foundings">
</div>


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/github_username/repo_name.svg?style=for-the-badge
[contributors-url]: https://github.com/JoseMaese
[forks-shield]: https://img.shields.io/github/forks/github_username/repo_name.svg?style=for-the-badge
[forks-url]: https://github.com/github_username/repo_name/network/members
[stars-shield]: https://img.shields.io/github/stars/github_username/repo_name.svg?style=for-the-badge
[stars-url]: https://github.com/github_username/repo_name/stargazers
[issues-shield]: https://img.shields.io/github/issues/github_username/repo_name.svg?style=for-the-badge
[issues-url]: https://github.com/github_username/repo_name/issues
[license-shield]: https://img.shields.io/github/license/github_username/repo_name.svg?style=for-the-badge
[license-url]: https://github.com/github_username/repo_name/blob/master/LICENSE.txt

<p align="right">(<a href="#readme-top">back to top</a>)</p>
