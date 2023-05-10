# Kora Tools

More tools coming soon!

## Xacro Generator

An easy was to create your robot's description files.

### Linux Usage

Put the kora_tools package into ~/my_ws/src.

Edit ~/my_ws/src/kora_tools/src/tools/xacro_generator.py according to your robot's requirements.

From the workspace root (~/my_ws) run ```colcon build```.

This will install xacro_generator.py into ros2 so you can run ```ros2 run kora_tools xacro_generator.py``` to put my_robot.urdf.xacro and my_robot.urdf into your robot's description folder.

From the workspace root (~/my_ws) run ```colcon build``` again to install my_robot.xacro.urdf and my_robot.urdf into your ROS2 workspace.

