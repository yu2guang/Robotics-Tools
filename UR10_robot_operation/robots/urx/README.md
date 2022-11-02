# UR10 Python Control (for GrassLab)

This library is used to control the UR10 Robot, Robotiq 3F gripper, and FT Sensor by Python scripts. The computer can directly control the robot to test everything easily. Modified from [python-urx](https://github.com/SintefManufacturing/python-urx) library and tested in GrassLab environment.


## Installation

1. Download this repository to any place you prefer, for example, in our case `~/urx`.
2. Add the path of this library to `PYTHONPATH`: `export PYTHONPATH=$HOME/urx:$PYTHONPATH`.

You can test whether it works by `import urx` in IPython.

## Before executing scripts

You must complete all the following steps before using Python script to control it. Otherwise, the gripper may not work. (the reason requires further investigation)

1. The computer to control the UR10 should be connected to WIFI `ROS_AP_5G`.
2. UR10 Control Box: Program Robot -> Load Program -> choose "Gripper_control_default.urp".
3. UR10 Control Box: Go to "Program" tab, "3F Activate", press "Execute now".
4. UR10 Control Box: Go to "Program" tab, "3F Pinch Close", press "Go to position" (set the position to make the gripper move)

Then you can use the sample script in the next section to test whether it works.

If you have exited the program and intended to run again, you should repeat the above steps, and choose "Reset and activate" option in step 3.


## A Sample Script


```python3
import urx
from urx.robotiq_three_finger_gripper import Robotiq_Three_Finger_Gripper
from urx.robotiq_force_torque import ForceTorque
import time

# Robot IP: refer to the control box: "Setup Robot" > "Network" > IP address
robot_ip = '192.168.0.119'

# Initialization
rob = urx.Robot(robot_ip)
gripper = Robotiq_Three_Finger_Gripper(rob)
ft = ForceTorque()

# Control UR10 Robot
cur_pose = rob.getl()  # [x, y, z, rx, ry, rz]
cur_joint = rob.getj()  # [j1, j2, j3, j4, j5, j6]
print('current pose: {}, current joint: {}'.format(cur_pose, cur_joint))
print('x, y, z = {}, {}, {}'.format(rob.x, rob.y, rob.z))  # however, rx, ry, rz is not working

## Blocking calls
acc = 0.01  # accelerator
vel = 0.01  # velocity
rob.movel((cur_pose[0], cur_pose[1], cur_pose[2] + 0.01, cur_pose[3], cur_pose[4], cur_pose[5]), acc, vel)  # absolute, linear move, blocking call
rob.movel((0, 0, 0.01, 0, 0, 0), acc, vel, relative=True)  # relative, linear move, blocking call
rob.movej(cur_joint, acc, vel)  # move using joint coord, blocking call

## Non-blocking call
rob.movel((-0.5, 0, 0, 0, 0, 0), acc, vel, relative=True, wait=False)  # relative, linear move, non-blocking call
time.sleep(0.1)
rob.stopl()  # stop after 0.1 sec

# Control Robotiq 3F Gripper
gripper.open_gripper()
gripper.close_gripper()
gripper.set_gripper(40)
gripper.set_gripper(255, speed=10)
gripper.set_gripper(0, speed=80, force=50)

# Get FT Sensor values
print('Current FT force values is: Fx = {}, Fy = {}, Fz = {}'.format(ft.Fx, ft.Fy, ft.Fz))
print('Current FT moment values is: Mx = {}, My = {}, Mz = {}'.format(ft.Mx, ft.My, ft.Mz))

```

For other examples and math3d-related methods, please refers to https://github.com/SintefManufacturing/python-urx

