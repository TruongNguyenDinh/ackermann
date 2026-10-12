# Repository Guidelines

## Project Structure &amp; Module Organization

This repository is a ROS 2 Humble workspace. Each top-level package is built independently with `colcon`:

- `ackermann_description` contains URDF/Xacro, launch files, RViz settings, and controller configuration.
- `ackermann_kinematics` and `ackermann_msg` are C++ packages; `ackermann_kinematics_py` is the Python kinematics package.
- `ackermann_odometry` and `ackermann_navigation` contain Python nodes, launch files, and Nav2/SLAM configuration. Their tests live in each package's `test/` directory.
- `small-warehouse-world` contains Gazebo worlds, models, maps, and launch files.
- `.devcontainer/` and `docker-compose.yml` define the ROS development container.

Keep new code, tests, and assets inside their owning package.

## Build, Test, and Development Commands

Run commands from the workspace root in a ROS Humble environment, such as the provided container. Source ROS before building:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build
source install/setup.bash
```

`colcon build` builds all discovered packages; `rosdep` installs their declared dependencies. Run package tests with `colcon test`; inspect results with `colcon test-result --verbose`. Launch a component with `ros2 launch <package> <launch_file>.launch.py` (for example, `ackermann_description model.launch.py`).

## Docker Environment

The ROS 2 development environment runs inside an existing Docker container on the current machine.

- Use the existing container named `elegant_jennings`.

- Start an interactive Bash session in the container with:

  \`\`\`bash

  docker exec -it elegant_jennings bash



## Coding Style &amp; Naming Conventions

Follow the existing ROS 2 package layout and declared build type (`ament_cmake` or `ament_python`). Use four spaces for Python indentation, `snake_case` for Python modules/functions, and package-specific names consistent with neighboring code. Name tests `test_*.py`. Keep configuration in the package's `config/` directory and robot descriptions in `urdf/`. Python packages include ament flake8, pep257, and copyright checks; run `colcon test` after changes.

## Testing Guidelines

Add or update tests in the affected package's `test/` directory. The Python packages use pytest through ament; CMake packages use ament lint checks where configured. For launch, URDF, or simulation changes, build the workspace and perform a relevant launch or simulation smoke check when the environment supports it.

## Commit &amp; Pull Request Guidelines

The checkout does not include readable Git history, so no repository-specific commit pattern can be confirmed. Use concise, descriptive commit messages. Follow `small-warehouse-world/CONTRIBUTING.md`: keep changes focused, discuss significant work in an issue, ensure tests pass, and include a clear PR description with reproduction steps or relevant issue links. Include screenshots for visible simulation or RViz changes.

## Mandatory Change Control

**Do not modify the existing project structure without asking for and receiving explicit approval from the user.**

- Do not create, delete, rename, move, or reorganize existing packages, directories, or files without prior approval.

- Do not change package dependencies, build systems, interfaces, or architectural boundaries without prior approval when these changes affect the existing project structure or design.

- Do not replace the current architecture with a different one without explicit approval.

- Prefer minimal, targeted changes that preserve the existing structure and functionality.

- Before making a change that requires structural modifications, explain:

  1. Why the change is necessary.

  2. Which files, directories, or packages will be affected.

  3. What alternatives are available.

  4. The expected impact on existing functionality.

- Wait for explicit user approval before proceeding with any unapproved structural modification.

- If the task cannot be completed without changing the structure, stop and ask the user for permission.

- Do not interpret a general request to fix a bug, implement a feature, or improve the code as permission to restructure the project.

These restrictions apply to all source code, ROS 2 packages, URDF/Xacro files, launch files, configuration files, Docker files, and development environment settings.