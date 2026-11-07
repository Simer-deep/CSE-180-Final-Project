# CSE-180-Final-Project

This repository is a clean ROS 2 workspace for the CSE 180 final project.

## Prerequisites

- Install ROS 2 for your platform and use the ROS distribution required by
  your course or team.
- Install `colcon` and the ROS 2 development tools for that distribution.

## Create a package

From the repository root, source your ROS 2 installation (replace `jazzy` with
your installed distribution), then create a package inside `src/`. For example:

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg create --build-type ament_cmake my_package --dependencies rclcpp
```

Add your project's packages and code under `src/`.

## Build and run

Build the workspace from the repository root:

```bash
colcon build --symlink-install
source install/setup.bash
```

Build products are generated in `build/`, `install/`, and `log/`; they are
machine- and environment-specific and are intentionally not committed. After
cloning this repository, create your packages under `src/` and run the build
commands above.