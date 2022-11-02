import os, can, sys, time, serial

import joblib
import numpy as np
import pandas as pd
from datetime import datetime


class TactileSensor:
    def __init__(self, dev='/dev/ttyUSB0@3000000', board_num=2, sensor_num=16, init_num=11,
                 csv_path=f'tactile_data_{datetime.now().strftime("%Y%m%d%H%M")}.csv',
                 dict_path=f'tactile_data_{datetime.now().strftime("%Y%m%d%H%M")}.pkl'):
        print('Connecting tactile sensors...')
        try:
            self.bus = can.interface.Bus(bustype='slcan',
                                         channel=dev,
                                         rtscts=True,
                                         bitrate=1000000)
        except serial.serialutil.SerialException as err:
            print(err)
            print('Run `sudo chmod 777 /dev/ttyUSB0` to avoid permission denied.')
            sys.exit(1)

        self.board_num = board_num
        self.sensor_num = sensor_num
        self.init_num = init_num
        self.csv_path = csv_path
        self.dict_path = dict_path
        self.sensor_arbit_id = [[1, 17, 33, 49, 65, 81, 97, 113, 257, 273, 289, 305, 321, 337, 353, 369],
                                [2, 18, 34, 50, 66, 82, 98, 114, 258, 274, 290, 306, 322, 338, 354, 370]]

        # stored dataframe & dict
        df_names = [f'x{j}_{i}' for i in range(self.board_num) for j in range(self.sensor_num)] + \
                   [f'y{j}_{i}' for i in range(self.board_num) for j in range(self.sensor_num)] + \
                   [f'z{j}_{i}' for i in range(self.board_num) for j in range(self.sensor_num)]
        df_names += ['data_type', 'timestamp']
        self.force_df = pd.DataFrame(columns=df_names)
        self.force_dict = {'tactile_data_np': [], 'data_type': [], 'timestamp': []}

        # get initial force matrix
        self.init_force_mat = np.zeros((self.board_num, self.sensor_num, 3))
        init_force_mat_list = []
        for i in range(self.init_num):
            tmp_force_mat, _ = self.get_complete_force_matrix(name='initial')
            init_force_mat_list.append(tmp_force_mat)
        self.init_force_mat = np.median(np.array(init_force_mat_list), axis=0)
        self.store_data(self.init_force_mat, time.time(), name='initial_finish')
        print('>>>>>>>>>>>>> Tactile sensor initialization finished. <<<<<<<<<<<<<<')

    def clear_buffer(self, num=9):
        for i in range(num):
            self.get_complete_force_matrix(store=False)

    def update_force(self, force_mat, arbit_id, data):
        """
        Update one force sensor value at each time.

        :param force_mat: (self.board_num, self.sensor_num, 3) np array
        :param arbit_id: msg.arbitration_id
        :param data: msg.data
        :return:

        # byte_number = msg.dlc
        # data = "".join("{:02X} ".format(byte) for byte in msg.data)
        """
        # get sensor numbers
        cur_board_num = (arbit_id % 2) ^ 1
        sda_num = arbit_id // 16 // 16 % 16
        taxel_num = arbit_id // 16 % 16
        pixel_num = sda_num * 8 + taxel_num

        # get sensor force data
        force_mat[cur_board_num, pixel_num, 0] = data[1] * 256 + data[2]  # x_axis
        force_mat[cur_board_num, pixel_num, 1] = data[3] * 256 + data[4]  # y_axis
        force_mat[cur_board_num, pixel_num, 2] = data[5] * 256 + data[6]  # z_axis

        return force_mat

    def get_complete_force_matrix(self, name='current', store=True):
        """
        Update complete force data matrix (2, 16, 3), and defined as the same record time.

        :return:
        """
        tmp_force_mat = np.zeros_like(self.init_force_mat)
        tmp_id_list = []

        while True:
            msg = self.bus.recv()
            arbit_id = msg.arbitration_id
            data = msg.data
            if arbit_id not in tmp_id_list:
                tmp_force_mat = self.update_force(tmp_force_mat, arbit_id, data)
                tmp_id_list.append(arbit_id)
                if len(tmp_id_list) == (self.board_num * self.sensor_num):
                    record_time = msg.timestamp
                    break

        # store current data
        if store:
            self.store_data(tmp_force_mat, record_time, name)

        return tmp_force_mat, record_time

    def store_data(self, force_mat, record_time, name='current'):
        # print(f'{record_time}: \'{name}\' saved.')

        # store data to dataframe
        cur_data = force_mat.reshape(-1, 3).T.flatten().tolist()
        cur_data += [name, record_time]
        cur_data_series = pd.Series(cur_data, index=self.force_df.columns)
        self.force_df = self.force_df.append(cur_data_series, ignore_index=True)

        # store data to dict
        self.force_dict['tactile_data_np'].append(force_mat)
        self.force_dict['timestamp'].append(record_time)
        self.force_dict['data_type'].append(name)

    def save_data(self):
        # save as csv
        if os.path.exists(self.csv_path):
            os.remove(self.csv_path)
        self.force_df.to_csv(self.csv_path, index=False, mode='a')
        print(f'{self.csv_path} saved.')

        # save as pkl
        joblib.dump(self.force_dict, self.dict_path)
        print(f'{self.dict_path} saved.')

    def process_end(self):
        self.bus.shutdown()


if __name__ == '__main__':
    # Tactile Sensor
    ts = TactileSensor()
    i = 0
    while True:
        cmd = input('\nProceed? (y/n)')
        if cmd == 'n':
            break
        ts.clear_buffer()
        for _ in range(20):
            force_mat, rtime = ts.get_complete_force_matrix()
            print(f'{i} ({rtime}): {force_mat[1][i][2]}')
        i += 1

    ts.save_data()
    ts.process_end()
