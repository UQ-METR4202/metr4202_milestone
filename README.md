# metr4202_milestone

The `metr4202_milestone` ROS package aims to provide feedback on the inverse
kinematics for a 4-DOF robot. IT DOES NOT VERIFY THE SOLUTION TO THE INVERSE
KINEMATICS. Students are encouraged to use the program in this package to ensure
different cases are handled for the METR4202 project. Run the program alongside
your inverse kinematics node and let the program send commands to it.

**CAUTION:** If collision is not handled, your robot will try to kill itself!
Be prepared to disable the motors during task 3.

---

## Requirement

- Your inverse kinematics node must subscribe to a topic of type
`geometry_msgs/Pose`.
- It must not rely on the orienation component of the pose
(talk to tutors if this is a problem).

---

## Install

1. Clone the repository into your catkin workspace source directory.

```SH
git clone https://github.com/UQ-METR4202/metr4202_milestone ~/catkin_ws/src
```

2. Before building the program, open the configuration header file under
`include/metr4202_milestone/config.hpp`. Make sure to read and adjust the file
according to your setup. Changing this file will require re-building.

3. Build the package and install it. This may take up to a minute.

```SH
cd ~/catkin_ws
catkin build # or catkin_make
source devel/setup.bash
```

---

## Usage

Launch the Dynamixel interface and your inverse kinematics node. Your stack must
eventually publish to the `/desired_joint_states` topic for the `dynamixel_interface_controller_node` to actuate the motors.

```SH
# Terminal 1
roslaunch dynamixel_interface dynamixel_interface_controller.launch
```

```SH
# Terminal 2
# Run or launch your inverse kinematics
```

```SH
# Terminal 3
rosrun metr4202_milestone milestone
```
