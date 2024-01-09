#! /usr/bin/env python3
import serial
import time
import pandas as pd
import numpy as np
from io import StringIO
import rclpy
import threading
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String
import socket

# Parameters
localPort=8888
bufferSize=1024


class esp32Communication(Node):
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=0.1):
        self.esp32 = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        super().__init__("control")
        self.susbcription = self.create_subscription(
            String, 
            '/movement',
            self.listener_callback, 
            10
        )
        self.timer = None
        self.timer_enabled = False  # Bandera para controlar el temporizador
        self.task_completed = False  # Bandera para indicar si la tarea ha terminado
        self.last_message = ""
        self.actual_pos = np.zeros(6)
        self.publisher_1 = self.create_publisher(Point, '/angles', 10)
        self.publisher_2 = self.create_publisher(Twist, '/command', 10)

    def get_input(self):
        # Read user input from the console
        input_values = input("Enter comma-separated values for linear x, y, z and angular x, y, z: ")
        try:
            # Convert the input values to floats
            values = [float(val.strip()) for val in input_values.split(',')]
            return values
        except ValueError:
            print("Invalid input. Please enter numeric values separated by commas.")
            return None
        
    def read_port(self):    
        packet = self.esp32.readline()
        return packet.decode('utf-8').rstrip('\n').rstrip('\r')
    
    def publish_twist(self):
        values = self.get_input()
        if values:
            twist_message = Twist()
            twist_message.linear.x, twist_message.linear.y, twist_message.linear.z, \
            twist_message.angular.x, twist_message.angular.y, twist_message.angular.z = values
            self.publisher_2.publish(twist_message)
    
    def file_treat(self, data_recieve):
        data = data_recieve.split(';')
        data = [float(element.replace(',', '.')) for element in data]
        result = tuple(data)
        self.angle_1 = result[0]
        self.angle_2 = result[1]
        return self.angle_1, self.angle_2

    def publish_angles(self):
        value = self.read_port()
        point_message = Point()
        point_message.x, point_message.y = self.file_treat(value)
        self.publisher_1.publish(point_message)
    
    def listener_callback(self, msg):
        self.move = msg.data
        if self.move == 'walk':
            self.handle_case1()
        elif self.move == 'march':
            self.handle_case2()
        elif self.move == 'pose0':
            self.handle_case3()
        elif self.move == 'pose2':
            self.handle_case4()
        else:
            self.handle_default()
        
    def handle_case1(self):
        vel = 2000  # lower is faster 1000 - 2000 us
        angle_init = 20
        k1 = 1.5
        k2 = 0.5
        k3 = 0.3

        self.matrixw = np.array([
            [1, angle_init, vel, 0, k1*angle_init, vel],
            [0, angle_init*k2, vel, 0, k2*angle_init, vel],
            [0, angle_init*(k1-k3), vel, 0, 0, 0],
            [0, angle_init*k3, vel, 1, k2*angle_init, vel],
            [0, 0, 0, 1, k1*angle_init, vel],
            [1, angle_init*k3, vel, 0, angle_init, vel],
            [1, angle_init*(1- k3 + k2), vel, 0, k2*angle_init, vel],
            [1, angle_init*(k2), vel, 1, angle_init*k3, vel]
        ])

        for row in self.matrixw:
            self.posew = ','.join(map(str, row))
            self.execute_subroutine(self.posew)
            
            # Obtener el tiempo de espera dinámico para la fila actual
            wait_time = self.calculate_wait_time(row)
            
            # Esperar el tiempo dinámico antes de la siguiente fila
            time.sleep(wait_time)

        # Reiniciar la bandera de tarea completada
        self.task_completed = False

    
    def handle_case2(self):
        print("case 2")

    def handle_case3(self):
        # matrix de posicion
        self.addr = 0
        self.theta1 = 20
        self.theta2 = -self.theta1
        self.vel = 20000
        self.matrixp0 = np.array([self.addr, self.theta1, self.vel, self.addr, self.theta2, self.vel])
        self.task_completed = False
        self.pose1 = ','.join(map(str, self.matrixp0))
        # Ejecutar subrutina 1
        self.execute_subroutine(self.pose1)
        time.sleep(2)

    def handle_case4(self):
        # matrix de posicion
        self.addr = 1
        self.theta1 = 20
        k = 0.6
        self.theta2 = k*self.theta1
        self.vel = 20000
        self.matrixp1 = np.array([self.addr, self.theta1, self.vel, not self.addr, self.theta2, self.vel])
        self.task_completed = False
        self.pose1 = ','.join(map(str, self.matrixp1))
    
        # Ejecutar subrutina 1
        self.execute_subroutine(self.pose1)
        time.sleep(2)

    def handle_default(self):
        self.stay = "0,0,0,0,0,0"
        self.execute_subroutine(self.stay)
    
    def execute_subroutine(self, data):
        self.send_data_with_timer(data)

    def send_data_with_timer(self, data):
        if self.timer is not None and self.timer.is_alive():
            self.timer.cancel()  # Cancelar temporizador existente si hay alguno

        self.esp32.write(data.encode())

        wait_time = self.calculate_wait_time(data)
        
        # Iniciar un nuevo temporizador
        self.timer = threading.Timer(wait_time, self.timer_callback)
        self.timer.start()

    def calculate_wait_time(self, data):
        # Implementa la lógica para calcular el tiempo de espera según la subrutina
        # Puedes ajustar esto según tus necesidades específicas
        return 10.0

    def timer_callback(self):
        print("Tarea completada")
        # Establecer la bandera de tarea completada
        self.task_completed = True

    def close(self):
        self.esp32.close()



def main(args = None):
    rclpy.init(args = args)
    node = esp32Communication()
    while rclpy.ok():
        #   node.publish_twist()
        node.publish_angles()
        rclpy.spin_once(node, timeout_sec=0.01)
    node.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
