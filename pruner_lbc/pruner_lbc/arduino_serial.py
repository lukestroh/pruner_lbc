#!/usr/bin/env python3
"""
Arduino Publisher node
"""

from typing import List
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter

from pruner_lbc_msgs.msg import IMUData, TOFData

import serial
import json


class ArduinoNode(Node):
    def __init__(self) -> None:
        super().__init__(self, node_name = "arduino_node")
        self.imu_pub = self.create_publisher(msg_type=IMUData, topic="imu_data")
        self.tof_pub = self.create_publisher(msg_type=TOFData, topic="tof_data")
        self.pub_timer = self.create_timer(timer_period_sec=0.001, callback=self.pub_callback)

        self.ser = self._connect_to_serial(port='com17', baudrate=115200)
        return

    def _connect_to_serial(self, port: str, baudrate: int) -> serial.Serial:
        """Connect to serial port"""
        return serial.Serial(port, baudrate)
    
    def pub_callback(self):
        imu_msg = IMUData()
        tof_msg = TOFData()

        
        data = self.get_serial_data()
        if data is None:
            return
        else:
            imu_msg.accelx = data["lsm.a.x"]
            imu_msg.accely = data["lsm.a.y"]
            imu_msg.accelz = data["lsm.a.z"]
            imu_msg.gyrox = data["lsm.g.x"]
            imu_msg.gyroy = data["lsm.g.y"]
            imu_msg.gyroz = data["lsm.g.z"]

            tof_msg.tof0_d = data["tof0"]
            tof_msg.tof1_d = data["tof1"]

            self.imu_pub.publish(imu_msg)
            self.tof_pub.publish(tof_msg)
        return

    def get_serial_data(self) -> dict:
        try:
            data_raw = self.ser.readline()
        except:
            return None
        return json.loads(data_raw)


def main():
    rclpy.init()

    arduino_node = ArduinoNode()

    rclpy.spin(arduino_node)

    arduino_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()