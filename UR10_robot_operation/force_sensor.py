from ur10_robot import UR10Robot
from robots.urx.robotiq_force_torque import ForceTorque


def get_ft_data():
    robot = UR10Robot()
    robot.gripper.set_ft_zero()
    ft = ForceTorque()
    ft_data = ft.fetch_all()
    robot.close()

    return ft_data


if __name__ == '__main__':
    # Force Torque Sensor
    cur_ft_data = get_ft_data()
    print(f'fx, fy, fz, mx, my, mz: {cur_ft_data}')
