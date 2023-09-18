# Kora Tools

More tools coming soon!

## Xacro Generator

An easy was to create your robot's description files.

### Linux Usage

Put the kora_tools package into ~/my_ws/src.

Edit ~/my_ws/src/kora_tools/src/tools/xacro_generator.py according to your robot's requirements.

From the workspace root (~/my_ws) run ```colcon build```.

This will install the updated xacro_generator.py into ros2 so you can run ```ros2 run kora_tools xacro_generator.py``` to put my_robot.urdf.xacro and my_robot.urdf into your robot's description folder.

From the workspace root (~/my_ws) run ```colcon build``` again to install my_robot.xacro.urdf and my_robot.urdf into your ROS2 workspace.

## SDF Generator

An easy way to create your robot's simulation files.

### Linux Usage

Put the kora_tools package into ~/my_ws/src.

Edit ~/my_ws/src/kora_tools/src/tools/sdf_generator.py according to your robot's requirements.

From the workspace root (~/my_ws) run ```colcon build```.

This will install your updated sdf_generator.py into ros2 so you can run ```ros2 run kora_tools sdf_generator.py``` to put your new SDF into your robot's description or simulation folder.

From the workspace root (~/my_ws) run ```colcon build``` again to install the generated into your ROS2 workspace.

## Get Inertials

Script to use PyMeshLab to scan through mesh files and put the data into a csv.

### Linux Usage

You're probably going to want to scan through the mesh files in your visuals folder, so edit the path appropriately. Additionally, you'll need to make a `geometry` folder in your robot's description folder.

You of course can't get the mass from MeshLab so this value is defaulted to 1 so that it's at least usable for testing purposes, so you'll have to replace these manually for your robot. 

This won't work if you're using Anaconda, so your might want to run it as `python3 get_inertials.py` instead of using `ros2 run kora_tools get_inertials.py` unless your ros2 installation doesn't use Anaconda.
