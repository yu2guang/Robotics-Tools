# Adpated from: https://github.com/ROBOTIS-GIT/emanual/tree/master/docs/en/software/dynamixel/dynamixel_sdk/sample_code/python

from dynamixel_sdk import *  # Uses Dynamixel SDK library
import numpy as np


def twosComplement_hex(hex_str, bits=16):
    signed_int = int(hex_str, bits)
    if signed_int & (1 << (bits-1)):
        signed_int -= (1 << bits)
    return signed_int


class dynamixel_motor_XM430_W350_T:
    def __init__(self, baud_rate=1000000, dxl_id=14,
                 device_name='/dev/ttyUSB1', dxl_moving_status_threshold=3):
        #********* DYNAMIXEL Model definition (XM430-W350-T) *********
        #***** (Use only one definition at a time) *****
        # control table reference: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode

        # Control table address
        self.addr_operating_mode = 11
        self.addr_torque_enable = 64
        self.addr_goal_current = 102
        self.addr_goal_position = 116
        self.addr_present_current = 126
        self.addr_present_position = 132
        self.dxl_minimum_position_value = 0         # refer to the minimum position limit of product emanual
        self.dxl_maximum_position_value = 4095      # refer to the maximum position limit of product emanual
        self.baud_rate = baud_rate

        # DYNAMIXEL Protocol Version (1.0 / 2.0)
        # https://emanual.robotis.com/docs/en/dxl/protocol2/
        self.protocol_version = 2.0

        # Factory default ID of all DYNAMIXEL is 1
        self.dxl_id = dxl_id

        # Use the actual port assigned to the U2D2.
        # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
        self.device_name = device_name

        self.operating_mode_dict = {'Current': 0, 'Velocity': 1, 'Position': 3,
                                    'Extended Position Control': 4, 'Current-based Position': 5, 'PWM': 16}
        self.operating_mode_value_dict = {0: 'Current', 1: 'Velocity', 3: 'Position',
                                          4: 'Extended Position Control', 5: 'Current-based Position', 16: 'PWM'}
        self.torque_enable = 1     # value for enabling the torque
        self.torque_disable = 0     # value for disabling the torque
        self.dist_position_fit = np.polyfit(
            np.array([0.004, 0.0145, 0.029, 0.0485, 0.069, 0.083, 0.089]),
            np.array([450, 736, 1023, 1279, 1537, 1793, 2048]), 6)
        self.position_dist_fit = np.polyfit(
            np.array([2048, 2098, 2148, 2198, 2248, 2298, 2348, 2398, 2448, 2498,
                      2548, 2598, 2648, 2698, 2748, 2798, 2848, 2898, 2948, 2998,
                      3048, 3098, 3148, 3198, 3248, 3298, 3348]),
            np.array([0.08969, 0.08958, 0.08831, 0.08774, 0.08618, 0.08350, 0.08116, 0.07895, 0.07648, 0.07261,
                      0.06953, 0.06584, 0.06165, 0.05788, 0.05398, 0.04952, 0.04563, 0.04243, 0.03819, 0.03475,
                      0.03173, 0.02820, 0.02522, 0.02223, 0.01960, 0.01699, 0.01576]), 26)
        self.dxl_moving_status_threshold = dxl_moving_status_threshold #20    # dynamixel moving status threshold

        self.comm_success = 0     # communication success result value

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.device_name)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(self.protocol_version)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.baud_rate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

    def success(self, dxl_comm_result, dxl_error):
        if dxl_comm_result != self.comm_success:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            return False
        else:
            return True

    def ping(self):
        # Try to ping the Dynamixel
        # Get Dynamixel model number
        dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, self.dxl_id)
        if self.success(dxl_comm_result, dxl_error):
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (self.dxl_id, dxl_model_number))

    def enable_torque(self, enable=True):
        if enable:
            dxl_comm_result, dxl_error = \
                self.packetHandler.write1ByteTxRx(self.portHandler, self.dxl_id, self.addr_torque_enable, self.torque_enable)
            if self.success(dxl_comm_result, dxl_error):
                print("Dynamixel has been successfully connected")
        else:
            dxl_comm_result, dxl_error = \
                self.packetHandler.write1ByteTxRx(self.portHandler, self.dxl_id, self.addr_torque_enable, self.torque_disable)
            self.success(dxl_comm_result, dxl_error)

    def write(self, address, data, byte_size):
        if byte_size == 1:
            dxl_comm_result, dxl_error = \
                self.packetHandler.write1ByteTxRx(self.portHandler, self.dxl_id, address, data)
        elif byte_size == 2:
            dxl_comm_result, dxl_error = \
                self.packetHandler.write2ByteTxRx(self.portHandler, self.dxl_id, address, data)
        else:
            dxl_comm_result, dxl_error = \
                self.packetHandler.write4ByteTxRx(self.portHandler, self.dxl_id, address, data)
        return self.success(dxl_comm_result, dxl_error)

    def read(self, address, byte_size):
        if byte_size == 1:
            dxl_data, dxl_comm_result, dxl_error = \
                self.packetHandler.read1ByteTxRx(self.portHandler, self.dxl_id, address)
        elif byte_size == 2:
            dxl_data, dxl_comm_result, dxl_error = \
                self.packetHandler.read2ByteTxRx(self.portHandler, self.dxl_id, address)
        else:
            dxl_data, dxl_comm_result, dxl_error = \
                self.packetHandler.read4ByteTxRx(self.portHandler, self.dxl_id, address)
        return self.success(dxl_comm_result, dxl_error), dxl_data

    def set_operating_mode(self, mode='Position'):
        self.write(self.addr_operating_mode, self.operating_mode_dict[mode], 1)

    def get_operating_mode(self):
        success, operating_mode = self.read(self.addr_operating_mode, 1)
        if success:
            print("[ID:%03d] PresOpMode:%d (%s Control Mode)" % (self.dxl_id, operating_mode, self.operating_mode_value_dict[operating_mode]))

    def set_goal_current(self, dxl_goal_current):
        self.write(self.addr_goal_current, dxl_goal_current, 2)

    def set_goal_position(self, dxl_goal_position):
        self.write(self.addr_goal_position, dxl_goal_position, 4)

    def get_present_current(self):
        success, current = self.read(self.addr_present_current, 2)
        current = twosComplement_hex(hex(current))
        if success:
            print("[ID:%03d] PresCur:%03d" % (self.dxl_id, current))
        return current

    def get_present_position(self):
        success, position = self.read(self.addr_present_position, 4)
        if success:
            print("[ID:%03d] PresPos:%03d" % (self.dxl_id, position))
        return position

    def dist2position(self, dist):
        """
        :param dist: 0.004 ~ 0.089 (m)
        :return: gripper_val: 450 ~ 2048 (pinch mode)
        """
        return int(round(np.polyval(self.dist_position_fit, dist)))

    def open_gripper_by_position(self, goal_position=2048):
        """
        :param goal_position: 0~4095
            +/-180: 0, -90: 1023, 0: 2048, 90: 3073
        :return:
        """
        self.enable_torque(enable=False)
        self.set_operating_mode('Position')
        self.get_operating_mode()
        self.enable_torque()
        self.set_goal_position(goal_position)
        while True:
            present_position = self.get_present_position()
            if abs(goal_position - present_position) <= self.dxl_moving_status_threshold:
                break

    def open_gripper_by_current(self, goal_current):
        """
        :param goal_current: -1193~1193 (unit: 2.69mA)
        :return:
        """
        self.enable_torque(enable=False)
        self.set_operating_mode('Current')
        self.get_operating_mode()
        self.enable_torque()
        self.set_goal_current(goal_current)
        while True:
            present_current = self.get_present_current()
            if abs(goal_current - present_current) <= self.dxl_moving_status_threshold:
                break

    def process_end(self):
        # Close port
        self.portHandler.closePort()


def dxl_position_control():
    gripper_motor = dynamixel_motor_XM430_W350_T(device_name='/dev/ttyUSB0')

    # set operating mode as 'Position Control'
    gripper_motor.enable_torque(enable=False)
    gripper_motor.set_operating_mode('Position')
    gripper_motor.get_operating_mode()

    # rotate motor to desired position: 0~4095
    # +/-180: 0, -90: 1023, 0: 2048, 90: 3073
    # [2048, 1023, 450]
    gripper_motor.enable_torque()
    while True:
        position = input('\nPlease enter the desired position (0~4095, press \"q\" to quit): ')
        if position == 'q':
            break
        goal_position = int(position)
        gripper_motor.set_goal_position(goal_position)
        while True:
            present_position = gripper_motor.get_present_position()
            if abs(goal_position - present_position) <= gripper_motor.dxl_moving_status_threshold:
                break
    gripper_motor.enable_torque(enable=False)

    gripper_motor.process_end()


def dxl_current_control():
    gripper_motor = dynamixel_motor_XM430_W350_T(device_name='/dev/ttyUSB0')

    # set operating mode as 'Current Control'
    gripper_motor.enable_torque(enable=False)
    gripper_motor.set_operating_mode('Current')
    gripper_motor.get_operating_mode()

    # let motor to desired current: -1193~1193 (unit: 2.69mA)
    gripper_motor.enable_torque()
    while True:
        current = input('\nPlease enter the desired current (-1193~1193 (unit: 2.69mA), press \"q\" to quit): ')
        if current == 'q':
            break
        goal_current = int(current)
        gripper_motor.set_goal_current(goal_current)
        while True:
            present_current = gripper_motor.get_present_current()
            if abs(goal_current - present_current) <= gripper_motor.dxl_moving_status_threshold:
                break
    gripper_motor.enable_torque(enable=False)

    gripper_motor.process_end()


if __name__ == '__main__':
    gripper_motor = dynamixel_motor_XM430_W350_T()

    # ping the motor
    gripper_motor.ping()
    gripper_motor.process_end()

    # control motor by position
    dxl_position_control()

    # control motor by current
    dxl_current_control()
    while True:
        goal_current = int(input('\nPlease enter the desired current: '))
        if goal_current < 0:
            break
        gripper_motor.open_gripper_by_current(goal_current)
        time.sleep(1)
        gripper_motor.open_gripper_by_position()

    # end process
    gripper_motor.enable_torque(enable=False)
    gripper_motor.process_end()



