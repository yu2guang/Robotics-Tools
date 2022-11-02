#! /usr/bin/env python

import socket
import time



class ForceTorque(object):

    def __init__(self, host="192.168.0.119", port=63351):
        self.host = host
        self.port = port
        self.timeout = 15

    def read(self):
        self.s = socket.create_connection((self.host, self.port), timeout=self.timeout)
        while True:
            str = self.s.recv(4096)
            # print('len = {}\nstr = {}'.format(len(str), str))
            if not str:
                continue
            str = str.decode()
            start_pos = str.find('(')
            end_pos = start_pos + str[start_pos:].find(')')
            if end_pos < 0:
                continue
            sub_str = str[start_pos + 1:end_pos]
            self.sensor_data = [float(val.strip()) for val in sub_str.split(',')]

            break

        self.s.close()
        return self.sensor_data

    def fetch_all(self):
        return self.read()

    @property
    def Fx(self):
        return self.read()[0]

    @property
    def Fy(self):
        return self.read()[1]

    @property
    def Fz(self):
        return self.read()[2]

    @property
    def Mx(self):
        return self.read()[3]

    @property
    def My(self):
        return self.read()[4]

    @property
    def Mz(self):
        return self.read()[5]



if __name__ == '__main__':
    ft = ForceTorque()
    import numpy as np
    while True:
        # ft.read()
        # print('Fx = ', ft.Fx)
        print('Fx = ', ft.Fx, ' | Fy = ', ft.Fy, ' | Fy = ', ft.Fy, ' | Fz = ', ft.Fz)

        time.sleep(3)