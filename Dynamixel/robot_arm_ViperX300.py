import joblib
import numpy as np
import math3d as m3d
from collections import OrderedDict
from interbotix_xs_modules.arm import InterbotixManipulatorXS


class ViperX300:
    def __init__(self):
        self.robot = InterbotixManipulatorXS('vx300s', 'arm', 'gripper')
        # self.robot.arm.capture_joint_positions()
        self.pose_record = OrderedDict()

    def exec_gripper(self, value=0.5, delay=2.0):
        # value >= 0, open; cmd < -0, close
        # print(f'value: {value}, gripper_value: {np.sign(value)*self.robot.gripper.gripper_value}')
        # self.robot.gripper.set_pressure(np.abs(value))  # set value in [0, 1]
        self.robot.gripper.gripper_controller(np.sign(value) * self.robot.gripper.gripper_value, delay)

    def set_relative_pose(self, target_pose):
        cur_pose_m3d = m3d.Transform(self.get_current_pose())
        target_pose_m3d = m3d.Transform(target_pose)

        relative_rads = target_pose_m3d.orient.to_euler('XYZ') - cur_pose_m3d.orient.to_euler('XYZ')
        relative_pos = target_pose_m3d.pos - cur_pose_m3d.pos

        print('cur_pose: {}, {}'.format(np.degrees(cur_pose_m3d.orient.to_euler('XYZ')), cur_pose_m3d.pos))
        print('target_pose: {}, {}'.format(np.degrees(target_pose_m3d.orient.to_euler('XYZ')), target_pose_m3d.pos))
        print('move_pose: {}, {}'.format(np.degrees(relative_rads), relative_pos))

        success = self.robot.arm.set_ee_cartesian_trajectory(x=relative_pos[0], y=relative_pos[1], z=relative_pos[2],
                                                             roll=relative_rads[0], pitch=relative_rads[1],
                                                             yaw=relative_rads[2])
        print(success)

    def set_joints(self, joints):
        self.robot.arm.publish_positions(joints)

    def set_pose(self, target_pose):
        _, success = self.robot.arm.set_ee_pose_matrix(target_pose)#,
                                                       # custom_guess=[0.003067961661145091, -0.452524334192276,
                                                       #               1.3330293893814087,
                                                       #               -0.003067961661145091, -0.8160778284072876, 0.0])
        print(success)

    def get_current_pose(self):
        return self.robot.arm.get_ee_pose()

    def add_current_pose(self, key):
        cur_pose = self.robot.arm.get_ee_pose()
        cur_joint_pos = self.robot.arm.joint_commands
        self.pose_record[key] = {'joint_position': cur_joint_pos, 'pose': cur_pose}
        print('Pose {} added'.format(key))
        return self.pose_record[key]

    def save_poses(self, file):
        joblib.dump(self.pose_record, file)
        print('Robot Poses Saved')

    def restore_poses(self, file):
        self.pose_record = joblib.load(file)


if __name__ == '__main__':
    # initial
    robot = InterbotixManipulatorXS('vx300s', 'arm', 'gripper')
    robot.arm.capture_joint_positions()
    print('\nT_sb: ', robot.arm.get_ee_pose())
    print('joint_commands: ', robot.arm.joint_commands)

    # internal pose
    robot.arm.go_to_home_pose()  # up and become right angle  ### caution!!! It might fall.
    robot.arm.go_to_sleep_pose()  # looks like the thinker

    # get current T_sb pose
    T_sb = robot.arm.get_ee_pose()
    print('get_ee_pose:', T_sb)

    # save pose
    robot.arm.go_to_sleep_pose()
    T_sb = robot.arm.get_ee_pose()
    print('get_ee_pose:', T_sb)
    
    # set_ee_pose_matrix: set target pose by matrix
    T_sd = np.identity(4)
    T_sd[0, 3] = 0.3
    T_sd[1, 3] = 0
    T_sd[2, 3] = 0.2
    print(T_sd)
    _, success = robot.arm.set_ee_pose_matrix(T_sd)
    T_sb = robot.arm.get_ee_pose()
    print('get_ee_pose:', T_sb)
    
    # set_ee_pose_components: set target pose by x, y, z (m), roll, pitch, and yaw (rad)
    _, success = robot.arm.set_ee_pose_components(x=0.2, y=0.1, z=0.2, roll=1.0, pitch=1.5)
    
    # set_ee_cartesian_trajectory: set target pose by relative values x, y, z (m), roll, pitch, and yaw (rad)
    # moves in a straight line
    success = robot.arm.set_ee_cartesian_trajectory(z=-0.2)
    robot.arm.set_ee_cartesian_trajectory(x=-0.2)
    robot.arm.set_ee_cartesian_trajectory(z=0.2)
    robot.arm.set_ee_cartesian_trajectory(x=0.2)
    
    # rotate single joint
    robot.arm.set_single_joint_position("waist", np.pi / 2.0)
    robot.arm.set_single_joint_position("waist", -np.pi / 2.0)
    success = robot.arm.set_single_joint_position("wrist_rotate", -np.pi / 2.0)
    print(robot.arm.info_index_map)

    # gripper open/close
    robot.gripper.close(delay=2.0)  # sec
    robot.gripper.open(delay=2.0)
    robot.gripper.set_pressure(1.0)  # set value in [0, 1]
    robot.gripper.close(delay=2.0)
    robot.gripper.open(delay=2.0)
