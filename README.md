PROJECT 1

run the following command in the root directory (ros2_ws) to build package
```
colcon build --symlink-install
```

Then source ros in new terminal
```
source install/setup.bash
```

Run world package
```
 ros2 launch p1_world tb4_in_world.launch.py
```
Run controller in new terminal
```
ros2 run project1_control controller
```

For keyboard control:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_key
```

To just view the world.sdf file
cd into the worlds directory
```
gz sim world.sdf
```



Set up and fixed all of the package and folder struture and the colcon build works. -Avi

Notes from class
- package for project1
- package for project1_controller (later part of project)
- consider launch configs in world launch file
- pass launch configs to launch description
- declare the controller node in this file so the file can use the package
- Controller_node.py
  - rclpy library
  - define the node (import Node from rclpy.node)
  - new class called Project1Controller, basically a ros2 node with other methods


PROJECT 2
Real robot instructions (OU TurtleBot 4)

Build and source on desktop.
```
cd /home/lacs0000/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Bring up and verify robot-side ROS on the Raspberry Pi.
```
ssh student@<robot-name>.cs.nor.ou.edu
ros2 topic list
```

Confirm /scan, /tf, /odom exist
If missing:
```
turtlebot4-daemon-restart```
```
then ros2 topic list again
if still missing: ```ros2 launch turtlebot4_bringup robot.launch.py```

In a desktop terminal (not SSH), run school network setup.
```robot-setup.sh```

Enter robot name
Run the printed exports and daemon restart commands in that same desktop terminal

Verify desktop can see robot topics.
```ros2 topic list```

Confirm /scan, /tf, /odom
Verify command message type on your robot.
```ros2 topic info /cmd_vel```

If type is geometry_msgs/msg/TwistStamped: apply code snippets below
If type is geometry_msgs/msg/Twist: your current code can run without this conversion

Run your controller on desktop.
```ros2 run project1_control controller```
Optional keyboard override (matches your /cmd_vel_key pattern, stamped messages).
```ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r /cmd_vel:=/cmd_vel_key```
Important operational change

Do not use the simulation launch in tb4_in_world.launch.py for the physical robot.
That file includes tb4_simulation_launch.py, so it is sim-only.


ros2 pkg prefix turtlebot4_navigation
ls $(ros2 pkg prefix turtlebot4_navigation)/share/turtlebot4_navigation/launch/

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false

Mention the timeout stuff with avoid and escape, launch file, inverted difference for avoid calculation, TwistStamp change 