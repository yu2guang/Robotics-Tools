#! /usr/bin/env python


import logging
import os
import time

from robots.urx.urscript import URScript

"""
This is the implementation for Grass Lab Robotiq 3F gripper.
It is modified from robotiq_two_finger_gripper.py and lots of content has been changed.
Briefly speaking, our implementation references the implementation in https://grass8.cs.nctu.edu.tw/Robotics/ur_control,
for every action, we send the full rq_script.script and the action to perform to the robot server.
If you want to understand the details, you may refer to rq_script.script first to know how the script works first. The
Python interface is just a wrapper of the script.
"""


SOCKET_HOST = "192.168.0.101"
SOCKET_PORT = 30002

class RobotiqScript(URScript):

    def __init__(self):
        super(RobotiqScript, self).__init__()
        self._import_rq_script()

    def _import_rq_script(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        rq_script = os.path.join(dir_path, 'rq_script.script')

        with open(rq_script, 'r') as f:
            rq_script = f.read()
            self._set_script(rq_script)

    def _set_gripper_activate(self):
        msg = "rq_activate()"
        self.add_line_to_program(msg)

    # Gripper mode
    def _set_gripper_set_pinch(self):
        msg = "rq_set_pinch()"
        self.add_line_to_program(msg)

    def _set_gripper_set_wide(self):
        msg = "rq_set_wide()"
        self.add_line_to_program(msg)

    def _set_gripper_set_basic(self):
        msg = "rq_set_basic()"
        self.add_line_to_program(msg)

    def _set_gripper_set_scissor(self):
        msg = "rq_set_scissor()"
        self.add_line_to_program(msg)

    # Gripper movement
    def _set_gripper_set_force(self, value):
        msg = "rq_set_force({})".format(value)  # noqa
        self.add_line_to_program(msg)

    def _set_gripper_set_speed(self, value):
        msg = "rq_set_speed({})".format(value)  # noqa
        self.add_line_to_program(msg)

    def _set_gripper_set_position_wait(self, value):
        """This is blocking call"""
        msg = "rq_move_wait({})".format(value)  # noqa
        self.add_line_to_program(msg)

    def _set_gripper_set_position(self, value):
        """This is non blocking call"""
        msg = "rq_move({})".format(value)  # noqa
        self.add_line_to_program(msg)


class Robotiq_Three_Finger_Gripper(object):

    def __init__(self,
                 robot,
                 payload=0.85,
                 speed=255,
                 force=100):
        self.robot = robot
        self.payload = payload
        self.speed = speed
        self.force = force
        self.logger = logging.getLogger(u"robotiq")
        self.logger.setLevel(logging.DEBUG)

    def _get_new_urscript(self):
        """
        Set up a new URScript to communicate with gripper
        """
        urscript = RobotiqScript()

        # Wait on activation to avoid USB conflicts
        urscript._sleep(0.1)

        return urscript

    def gripper_activate(self):
        urscript = self._get_new_urscript()

        urscript._set_gripper_activate()

        # Send the script
        self.robot.send_program(urscript())

        # sleep the code the same amount as the urscript to ensure that
        # the action completes
        time.sleep(2.0)

    def gripper_action(self, pos, speed=None, force=None):
        """
        Activate the gripper to a given value from 0 to 255

        0 is open
        255 is closed
        """
        urscript = self._get_new_urscript()

        if not speed:
            speed = self.speed
        if not force:
            force = self.force

        # Move to the position
        sleep = 2.0
        urscript._set_gripper_set_force(force)
        urscript._set_gripper_set_speed(speed)
        urscript._set_gripper_set_position_wait(pos)
        urscript._sleep(sleep)

        # Send the script
        self.robot.send_program(urscript())

        # sleep the code the same amount as the urscript to ensure that
        # the action completes
        time.sleep(sleep)

    def open_gripper(self):
        self.gripper_action(0)

    def close_gripper(self):
        self.gripper_action(255)

    def set_gripper(self, pos, speed=None, force=None):
        self.gripper_action(pos, speed, force)
