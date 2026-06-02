# phi_p3dx_localization

This repository contains ROS2 packages for localization with the Pioneer P3-DX robot using particle filter (MCL - Monte Carlo Localization) in different environments: Gazebo simulation (3D), MobileSim (2D), and physical robot.

## Overview

The `phi_p3dx_localization` package provides:
- **Localization nodes**: Particle filter implementation for robot pose estimation.
- **Multi-platform compatibility**: Works in Gazebo, MobileSim, and physical robots.
- **Educational examples**: Commented code in Python and C++ for robotics students.

### Package Structure
- `scripts/`: Python scripts (e.g., `localization_example.py`).
- `src/`: C++ code (e.g., `localization_example.cpp`, `localization_node.cpp`).
- `include/`: C++ headers (e.g., `localization_node.hpp`).
- `launch/`: Launch files for different environments.
- `config/`: Configurations (e.g., RViz).

## Prerequisites

- **ROS2 Humble** (or compatible).
- **Gazebo** (for 3D simulation).
- **MobileSim** (for 2D simulation, if available).
- Dependencies: `rclcpp`, `rclpy`, `geometry_msgs`, `nav_msgs`, `tf2`, `tf2_geometry_msgs`.

## Installation

1. **Clone the repository** into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/phir2-lab/phi_p3dx_localization.git
   ```

2. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select phi_p3dx_localization
   source install/setup.bash
   ```

## Usage

### Gazebo Simulation (3D)

1. Launch the simulation:
   ```bash
   ros2 launch phi_p3dx_localization bringup_gazebo.launch.py map_name:=obstacles
   ```

2. Run the localization node (C++):
   ```bash
   ros2 run phi_p3dx_localization localization_example_cpp
   ```

   Or in Python:
   ```bash
   ros2 run phi_p3dx_localization localization_example_py
   ```

3. In RViz:
   - View particles visualization in `/particles` topic
   - Monitor estimated pose in `/estimated_pose` topic

### MobileSim Simulation (2D)

1. Launch the simulation:
   ```bash
   ros2 launch phi_p3dx_localization bringup_mobilesim.launch.py map_name:=obstacles
   ```

2. Run the localization node as above.

### Real Robot

1. Connect the Pioneer P3-DX robot.

2. Launch the system:
   ```bash
   ros2 launch phi_p3dx_localization bringup_robot.launch.py
   ```

3. Run the localization node.

## License

This project is distributed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.
