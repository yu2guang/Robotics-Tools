import numpy as np
from robots.urx.robot import Robot
from robots.urx.robotiq_three_finger_gripper import Robotiq_Three_Finger_Gripper
import math3d
import time


def main():
    robot = Robot('192.168.0.119')
    gripper = Robotiq_Three_Finger_Gripper(robot)  # Run script on control box first
    cur_pose = robot.get_pose()
    print(cur_pose.orient.array)  # 3*3 rotation matrix
    print(cur_pose.pos.array)  # [x,y,z]
    print(cur_pose.array)  # 4*4 transformation matrix
    new_pose = math3d.Transform()
    new_pose.orient = cur_pose.orient.array
    new_pose.pos = [0.3, 0.3, 0.2]
    print(new_pose.array)
    robot.set_pose(new_pose, 0.5, 0.2)

    transform_mat = new_pose.array  # numpy 4*4 transform matrix
    transform_mat[2, 3] += 0.3  # Z+=0.3
    new_pose_up = math3d.Transform(transform_mat) # From 4*4 transform matrix
    robot.set_pose(new_pose_up, 0.5, 0.2)

    new_pose_up.pos.z -= 0.2  # Set position x,y,z
    new_pose_up.pos.x -= 0.2
    robot.set_pose(new_pose_up, 0.5, 0.2, wait=False)
    while True:
        time.sleep(0.1)  # sleep first since the robot may not have processed the command yet
        print('Running')
        if robot.is_program_running():
            break
    robot.close()  # Must close the socket!


if __name__ == '__main__':
    main()
