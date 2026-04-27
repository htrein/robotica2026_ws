# phi_p3dx_planning

This repository contains ROS2 packages for motion planning with the Pioneer P3-DX robot in different environments: Gazebo simulation (3D), MobileSim (2D), and physical robot.

## Overview

The `phi_p3dx_planning` package provides:
- **Planning nodes**: Path planning implementations for navigation and obstacle avoidance.
- **Multi-platform compatibility**: Works in Gazebo, MobileSim, and physical robots.
- **Educational examples**: Commented code in Python and C++ for robotics students.

### Package Structure
- `scripts/`: Python scripts (e.g., `planning_example.py`).
- `phi_p3dx_planning/`: Python base class
- `src/`: C++ code (e.g., `planning_example.cpp`, `planning_node.cpp`).
- `include/`: C++ headers (e.g., `planning_node.hpp`).
- `launch/`: Launch files for different environments.
- `config/`: Configurations (e.g., RViz, SLAM, parameters).

## Prerequisites

- **ROS2 Humble** (or compatible).
- **Gazebo** (for 3D simulation).
- **MobileSim** (for 2D simulation, if available).
- Dependencies: `rclcpp`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `tf2`, `visualization_msgs`, `ros_gz_sim`, `ros_gz_bridge`.

## Installation

1. **Clone the repository** into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/phir2-lab/phi_p3dx_planning.git
   ```

2. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select phi_p3dx_planning
   source install/setup.bash
   ```

## Usage

### Gazebo Simulation (3D)

1. Launch the simulation:
   ```bash
   ros2 launch phi_p3dx_planning bringup_gazebo.launch.py map_name:=obstacles
   ```
2. Run the planning node (Python):
   ```bash
   ros2 run phi_p3dx_planning planning_example_py
   ```

   Or in C++:
   ```bash
   ros2 run phi_p3dx_planning planning_example_cpp
   ```

3. In RViz:
   - Use "2D Nav Goal" or custom topics to send goals.

### MobileSim Simulation (2D)

1. Launch the simulation:
   ```bash
   ros2 launch phi_p3dx_planning bringup_mobilesim.launch.py map_name:=obstacles
   ```

2. Run the planning node as above.

### Real Robot

1. Connect the Pioneer P3-DX robot.

2. Launch the system:
   ```bash
   ros2 launch phi_p3dx_planning bringup_robot.launch.py
   ```

3. Run the planning node.

## Alternative Branches

- **`mobilesim-only`**: A version without Gazebo support, focused only on MobileSim (2D) and real robot navigation. This branch removes Gazebo dependencies and launch files for users who prefer a simpler setup without 3D simulation.

  To use this branch:
  ```bash
  git checkout mobilesim-only
  ```

## License

This project is distributed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.
