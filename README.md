# ros_assignments
Assignments for RPwR course

# Requirements

- [CarmineD8/robot_description](https://github.com/CarmineD8/robot_description)
- `curses` available from Python (Ubuntu has built-in support for that)
- `libncurses` (use `apt install libncurses5-dev `)

Please copy appropriate folders in the ROS and ROS2 workspaces, assumed to be `$ROS_WS` and `$ROS2_WS`:
```
cp -r ros/ros_assignments $ROS_WS/src/
cp -r ros2/ros_assignments $ROS2_WS/src/
```

This `README` will not assume any sourced bashrc, hence all the required `source` commands will be reported.

# Assignment n. 1

## How to build

`catkin_make` the ROS package:
```
source /opt/ros/noetic/setup.bash
cd $ROS_WS
catkin_make
source $ROS_WS/devel/setup.bash
```

## How to run

0. Open a terminal `t0` and source the ROS environment (if not already done)
```
source /opt/ros/noetic/setup.bash
source $ROS_WS/devel/setup.bash
```

1. Run the simulation environment
```
roslaunch ros_assignments exercise.launch
```

2. On a separate terminal (sharing the same configuration as `t0`) run the interactive user interface:
```
rosrun ros_assignments user_interface.py
```
The Python user interface requires `curses`.

<p align="center"><img src="https://github.com/xEnVrE/ros_assignments/blob/master/assets/ui.jpg" alt="" height=300px/></p>


# Assignment n. 2

## Prerequisites

You will need two separate environments for ROS and ROS2.

The simulation environment will be hosted in the ROS environment. The position generation service and the user interface are made using ROS2 C++ nodes.

The user interface requires `libncurses`. Also, you need to clone `ros1_bridge` in the ROS2 environment:

```
cd $ROS2_WS
git clone https://github.com/ros2/ros1_bridge.git
```

## How to build

Please `catkin_make` the ROS environment first:
```
source /opt/ros/noetic/setup.bash
cd $ROS_WS
catkin_make
```

In order to build the ROS2 related part, in a new shell:
```
source /opt/ros/foxy/setup.bash
cd $ROS2_WS
colcon build --symlink-install --packages-skip ros1_bridge
```

Then proceed, in another shell:
```
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash 
source $ROS_WS/devel/setup.bash
source $ROS2_WS/install/setup.bash
cd $ROS2_WS
colcon build --packages-select ros1_bridge --cmake-force-configure
```

# How to run

0. Open a terminal `t0` and source the ROS environment (if not already done):
```
source /opt/ros/noetic/setup.bash
source $ROS_WS/devel/setup.bash
```

1. Run the simulation environment in `t0`:
```
roslaunch ros_assignments exercise.launch
```

2. Open a terminal `t1` and source the ROS/ROS2 hybrid environment (if not already done):
```
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash 
source $ROS_WS/devel/setup.bash
source $ROS2_WS/install/setup.bash
```

3. Run the ROS/ROS2 bridge in `t1`:
```
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

4. Open a terminal `t2` and source the ROS2 environment (if not already done):
```
source /opt/ros/foxy/setup.bash 
source $ROS2_WS/install/setup.bash
```

5. Run the position generator service from `t2`
```
ros2 run ros_assignments position_generation_server
```

6. On a separate terminal (sharing the same configuration as `t2`) run the interactive user interface:
```
ros2 run ros_assignments user_interface
```

