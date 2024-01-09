#! /usr/bin/env python3
import serial
import time
import pandas as pd
import numpy as np
from io import StringIO
import rclpy
from std_msgs.msg import String as StringMsg
from rclpy.node import Node
import socket
from threading import Timer

# Parameters
localPort = 8888
bufferSize = 1024


class esp32Communicationw(Node):
    
    def __init__(self, socketn = socket.AF_INET, socketd= socket.SOCK_DGRAM):
        super().__init__("control")
        self.esp32w = socket.socket(socketn, socketd)
        self.esp32w.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) #enable broadcasting mode
        self.esp32w.bind(('', localPort))
        self.publisher_1 = self.create_publisher(StringMsg, '/movement', 10)
        self.message = StringMsg()
        print("UDP server : {}:{}".format(self.get_ip_address(),localPort))
    
    def get_ip_address(self):
        """get host ip address"""
        ip_address = ''
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8",80))
        ip_address = s.getsockname()[0]
        s.close()
        return ip_address
        
    def get_wdata(self):
        data, addr = self.esp32w.recvfrom(1024)
        data_rev = data.decode('utf-8')
        data_rev = data_rev.split(';')
        data_rev = [float(element.replace(',', '.')) for element in data_rev]
        result = tuple(data_rev)
        print(result)
        stay_msg = StringMsg()
        stay_msg.data = 'stay'
        walk_msg = StringMsg()
        walk_msg.data = 'walk'
        if result[0] > 0.5:
            self.publisher_1.publish(stay_msg)
        elif result[1] > 0.5:
            self.publisher_1.publish(walk_msg)
        # print(data)
        # print(addr)
        #return data, addr


def main(args = None):
    rclpy.init(args = args)
    node = esp32Communicationw()
    while rclpy.ok():
        #   node.publish_twist()
        node.get_wdata()
    rclpy.spin(node)
    node.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()