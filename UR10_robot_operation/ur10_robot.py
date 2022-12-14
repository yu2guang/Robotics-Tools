import numpy as np
from robots.urx.robot import Robot
import math3d as m3d
from collections import OrderedDict
from robots.urx.robotiq_three_finger_gripper import Robotiq_Three_Finger_Gripper
import time, joblib

DEFAULT_ACC = 0.5
DEFAULT_VEL = 0.2


# DEFAULT_VEL = 1.
# 0.0013874173934951273 0.09858393669128418

# Some helper functions for robotic perception and control
class UR10Robot(Robot):
    def __init__(self, tcp='192.168.0.119', tableTrobot=None):  # GrassLab UR10
        """
        Useful functions

        get_pose()-> m3d.Transform() current pose
        moveL(pose,acc,vel)
        moveJ(pose,acc,vel)

        add_current_poses(key)
        save_poses(file_path)
        restore_poses(file_path)
        """
        super(UR10Robot, self).__init__(tcp)
        self.pose_record = OrderedDict()  # m3d.Transform type
        self.gripper = Robotiq_Three_Finger_Gripper(self)
        self.gripper_w_fit = np.polyfit(
            np.array([0.0, 0.01344, 0.02811, 0.04276, 0.05695, 0.07325, 0.0904, 0.10578, 0.12287, 0.13844, 0.1538, 0.16]),
            np.array([110, 100, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0]), 11)
        self.gripper_zdist_fit = np.polyfit(
            np.array([110, 100, 90, 80, 70, 55, 40, 30, 20, 10, 0]),
            np.array([0.112, 0.116, 0.12, 0.123, 0.125, 0.126, 0.125, 0.123, 0.12, 0.116, 0.112]), 10)
        # Optional for pose calculation
        self.tableTrobot = tableTrobot

    def moveL(self, pose, acc=DEFAULT_ACC, vel=DEFAULT_VEL):  # pose in m3d.Transform
        if isinstance(pose, np.ndarray):
            pose = m3d.Transform(pose)
        self.set_pose(pose, acc=acc, vel=vel, command='movel')

    def moveJ(self, pose, acc=DEFAULT_ACC, vel=DEFAULT_VEL):
        if isinstance(pose, np.ndarray):
            pose = m3d.Transform(pose)
        self.set_pose(pose, acc=acc, vel=vel, command='movej')

    def open_gripper(self, gripper_val):
        if gripper_val > 110:
            gripper_val = 110
        elif gripper_val < 0:
            gripper_val = 0

        self.gripper.set_gripper(gripper_val, speed=255, force=255)
        print(f'Set gripper position to {gripper_val}.')

    def get_gripper_val_by_dist(self, dist):
        """
        :param dist: 0.16 ~ 0.0 (m)
        :return: gripper_val: 0 ~ 110 (pinch mode)
        """
        return int(round(np.polyval(self.gripper_w_fit, dist)))

    def get_gripper_finger_zdist(self, gripper_val):
        """
        :param gripper_val: 0 ~ 110 (pinch mode)
        :return: finger zdist (m)
        """
        return np.polyval(self.gripper_zdist_fit, gripper_val)

    def rotate_gripper(self, rotate_deg, acc=DEFAULT_ACC, vel=DEFAULT_VEL, wait=True):
        """
        Rotate z-axis of gripper.

        :param rotate_deg:
        :param acc:
        :param vel:
        :param wait:
        :return: actual rotate degree, rotate time, end timestamp
        """
        cur_tcp = self.get_pose()
        start_degree = -np.degrees(cur_tcp.orient.to_euler('XYZ'))[2]

        cur_tcp.orient.rotate_zt(np.deg2rad(-rotate_deg))
        # goal_degree = -np.degrees(cur_tcp.orient.to_euler('XYZ'))[2]
        st = time.time()
        self.set_pose(cur_tcp, acc=acc, vel=vel, wait=wait)
        endt = time.time()

        cur_tcp = self.get_pose()
        end_degree = -np.degrees(cur_tcp.orient.to_euler('XYZ'))[2]

        return end_degree - start_degree, cur_tcp, endt - st, endt

    def rotate_gripper_smoothly(self, rotate_deg: float, thershold=0.5):
        cur_tcp = self.get_pose()
        cur_tcp.orient.rotate_zt(np.deg2rad(-rotate_deg))
        goal_degree = np.degrees(cur_tcp.orient.to_euler('XYZ'))[2]
        st = time.time()

        # if abs(rotate_deg) < 0.1:
        #     thershold = abs(rotate_deg)
        # rot_deg_each = 1 if abs(rotate_deg) > 1 else abs(rotate_deg)
        # while True:
        #     self.rotate_gripper(int(np.sign(rotate_deg))*rot_deg_each, wait=False)
        #     cur_tcp = self.get_pose()
        #     cur_degree = np.degrees(cur_tcp.orient.to_euler('XYZ'))[2]
        #     diff = abs(goal_degree - cur_degree)
        #     rot_deg_each = diff if rot_deg_each > diff else rot_deg_each
        #     if diff < thershold:
        #         endt = time.time()
        #         break

        if abs(rotate_deg) > 1:
            rot_deg_each = 1 if abs(rotate_deg) >= 1 else 0.1
            thershold = rot_deg_each * 0.5
            while True:
                self.rotate_gripper(int(np.sign(rotate_deg)) * rot_deg_each, wait=False)
                cur_tcp = self.get_pose()
                cur_degree = np.degrees(cur_tcp.orient.to_euler('XYZ'))[2]
                diff = -(goal_degree - cur_degree)
                print(f'rotate_deg: {rotate_deg}, rot_deg_each: {rot_deg_each}, diff:{diff}, threshold: {thershold}')
                if abs(diff) < 1:
                    break

                rot_deg_each = 0.1 if diff < 1 else 1
                thershold = rot_deg_each * 0.5

            self.rotate_gripper(rotate_deg)
            cur_tcp = self.get_pose()
            cur_degree = np.degrees(cur_tcp.orient.to_euler('XYZ'))[2]
            diff = -(goal_degree - cur_degree)
            endt = time.time()
        else:
            self.rotate_gripper(rotate_deg)
            cur_tcp = self.get_pose()
            cur_degree = np.degrees(cur_tcp.orient.to_euler('XYZ'))[2]
            diff = -(goal_degree - cur_degree)
            endt = time.time()

        return diff, endt - st

    def add_current_pose(self, key):
        self.pose_record[key] = self.get_pose()
        self.logger.debug('Pose {} added'.format(key))
        return key

    def del_pose(self, key):
        del self.pose_record[key]
        self.logger.debug('Pose {} deleted'.format(key))

    def list_pose(self):
        return self.pose_record.keys()

    def set_pose_by_key(self, key, acc=DEFAULT_ACC, vel=DEFAULT_VEL):
        try:
            pose = self.pose_record[key]
            self.set_pose(pose, acc=acc, vel=vel)
            self.logger.debug('Pose {} ok'.format(key))
            print(f'Move to \'{key}\'.')
        except KeyError:
            self.logger.debug('Pose {} Not Found'.format(key))

    def save_poses(self, file):
        joblib.dump(self.pose_record, file)
        self.logger.info("Robot Poses Saved")
        print(f'{file} saved.')

    def restore_poses(self, file):
        self.pose_record = joblib.load(file)
        return self.logger.info("Robot Poses Restored:{}".format(list(self.pose_record.keys())))

    def home(self):
        self.set_pose_by_key('Home')

    # def drop_fruit(self, dist, fruit_name):
    #     self.set_pose_by_key(fruit_name)
    #     self.open_gripper(dist)
    #     self.home()

    def move_1234(self, direct, dist):
        cur_tcp = self.get_pose()

        if direct == '1':
            cur_tcp.pos.z += dist
        elif direct == '2':
            cur_tcp.pos.z -= dist
        elif direct == '3':
            cur_tcp.pos.y += dist
        elif direct == '4':
            cur_tcp.pos.y -= dist

        self.moveL(cur_tcp)

    # def go_to_fruit(self, fruit_pose):
    #     # frp_inv = np.linalg.inv(np.array(fruit_pose.array))[0:-1, -1]
    #     # print(frp_inv)
    #     # frp *= -1
    #     # frp[0] -=  0.3
    #     # frp[2] += 0.25
    #
    #     cur_tcp = self.get_pose()
    #     cur_tcp = np.array(cur_tcp.array) + np.array([[0, 0, 0, 0.65], # 0.65
    #                                                   [0, 0, 0, 0.05], # 0.05
    #                                                   [0, 0, 0, -0.3], # -0.3
    #                                                   [0, 0, 0, 0]])
    #     cur_tcp = m3d.Transform(cur_tcp)
    #     self.moveL(cur_tcp)
    #     # print(cur_tcp)
    #     # self.moveL(cur_tcp)

    # @staticmethod
    # def calc_view_pose(vpt, in_plane=0, y_rotate=0, x_rotate=0):
    #     """
    #     :param vpt: Viewpoint (3) (Camera Translation in object centric frame)
    #     :param in_plane: in-plane rotation for camera(+z), radian
    #     :param y_rotate: 0 or angle, Rotate along camera -y axis
    #     :param x_rotate: 0 or angle, Rotate along camera +x axis
    #     :return: |R(3*3)  t(3*1)| Full pose in math3d Transform
    #              |0         1   |
    #
    #
    #     Camera Convention (opencv):
    #         +z toward object
    #         +y approximately downward
    #         +x approximately on the right
    #
    #         +z(toward object)---->+x
    #         |
    #         |
    #         |
    #         V
    #         +y
    #     """
    #     z = -np.array(vpt, dtype=np.float32)  # Forward direction (+z for Cam in Object)
    #     z /= np.linalg.norm(z)
    #     u = np.array([0.0, 0.0, 1.0])  # Object +z
    #     x = np.cross(z, u)  # (+x for Cam in Object)
    #     if np.count_nonzero(x) == 0:
    #         x = np.array([1.0, 0.0, 0.0])
    #     x /= np.linalg.norm(x)
    #     y = np.cross(z, x)
    #     y /= np.linalg.norm(y)
    #     R = np.hstack((x[None].T, y[None].T, z[None].T))  # oRc camera rotation in object frame
    #     """
    #     In-Plance Rotation
    #     """
    #     t = np.array(vpt)
    #     pose=m3d.Transform(R, t)
    #     # TODO: check in renderer
    #     if in_plane != 0:
    #         pose.orient.rotate_zt(in_plane)
    #     if y_rotate != 0:
    #         pose.orient.rotate_yt(-y_rotate)
    #     if x_rotate != 0:
    #         pose.orient.rotate_xt(x_rotate)
    #
    #     return pose


if __name__ == '__main__':
    # initial robot
    robot = UR10Robot()

    # get current pose
    cur_tcp = robot.get_pose()
    print('Current pose: rx, ry, rz, x, y, z')
    print(np.degrees(cur_tcp.orient.to_euler('XYZ')), cur_tcp.pos.array)

    # restore poses
    robot.restore_poses('stored_pose')

    # add current pose
    print(robot.list_pose())
    robot.add_current_pose('go')
    print(robot.list_pose())

    # save poses
    robot.save_poses('stored_pose')

    # move by transform tcpTbase
    pick_pose = np.array([[-0.73465557, -0.65088167,  0.19140073, 0.48181],
                          [-0.66823577,  0.74295929, -0.03837264, -0.18539],
                          [-0.1172269 , -0.15609149, -0.98076159, 0.08139],
                          [0, 0, 0, 1]])
    robot.moveL(pick_pose)

    # open/close the gripper with meters
    robot.open_gripper(0.04)

    # rotate the gripper by degree
    rotate_deg = float(input('Rotate degree: '))
    robot.rotate_gripper_smoothly(rotate_deg)

    # move by key
    robot.set_pose_by_key('home')

    # close the connection
    robot.close()
