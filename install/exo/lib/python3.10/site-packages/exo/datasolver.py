#! /usr/bin/env python3
import serial
import time
import pandas as pd
import numpy as np
from io import StringIO
import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String

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
        self.susbcription2 = self.create_subscription(String, 
                                                      '/mode', 
                                                      self.listener_callback2, 
                                                      10)
        self.susbcription3 = self.create_subscription(Point, '/slider', 
                                                      self.listener_callback3, 10)
        self.timer = None
        self.wait = 8.5
        self.angulos_guardados = []
        self.manual = False
        self.timer_enabled = False  # Bandera para controlar el temporizador
        self.task_completed = False  # Bandera para indicar si la tarea ha terminado
        self.actual_pos = np.zeros(6)
        self.publisher_1 = self.create_publisher(Point, '/angles', 10)
        self.publisher_2 = self.create_publisher(Point, '/command', 10)
        self.angles = Point()

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
        pass
    
    def file_treat(self, data_recieve):
        data = data_recieve.split(';')
        try:
            data = [float(element.replace(',', '.')) for element in data]
            result = tuple(data)
            self.angle_1 = result[0]
            self.angle_2 = result[1]
            return self.angle_1, self.angle_2
        except:
            print("angle lecture fail")

    def publish_angles(self):
        value = self.read_port()
        point_message = Point()
        try:
            point_message.x, point_message.y = self.file_treat(value)
            self.publisher_1.publish(point_message)
        except:
            pass

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
            signo = -1 if row[0] == 1 else 1
            angulo = signo * row[1]
            signo2 = 1 if row[3] == 1 else -1
            angulo2 = signo2 *row[4]
            # Guardar el ángulo
            self.angulos_guardados.append((angulo, angulo2))
            self.angles.x = angulo
            self.angles.y = angulo2
            self.publisher_2.publish(self.angles)
            self.posew = ','.join(map(str, row))
            self.execute_subroutine(self.posew)
            
            # Obtener el tiempo de espera dinámico para la fila actual
            wait_time = self.calculate_wait_time(row)
            
            # Esperar el tiempo dinámico antes de la siguiente fila
            time.sleep(wait_time)
        # print(self.angulos_guardados)
        # Reiniciar la bandera de tarea completada
        self.task_completed = False

    
    def handle_case2(self):
        vel2 = 1500  # lower is faster 1000 - 2000 us
        angle_init = 20
        k1 = 1.5
        k2 = 0.5
        k3 = 0.3

        self.matrixw = np.array([
            [1, angle_init, vel2, 0, k1*angle_init, vel2],
            [0, angle_init*k2, vel2, 0, k2*angle_init, vel2],
            [0, angle_init*(k1-k3), vel2, 0, 0, 0],
            [0, angle_init*k3, vel2, 1, k2*angle_init, vel2],
            [0, 0, 0, 1, k1*angle_init, vel2],
            [1, angle_init*k3, vel2, 0, angle_init, vel2],
            [1, angle_init*(1- k3 + k2), vel2, 0, k2*angle_init, vel2],
            [1, angle_init*(k2), vel2, 1, angle_init*k3, vel2]
        ])

        for row in self.matrixw:
            # Calcular el ángulo según el valor en la columna 0
            signo = -1 if row[0] == 1 else 1
            angulo = signo * row[1]
            signo2 = 1 if row[3] == 1 else -1
            angulo2 = signo2 *row[4]
            # Guardar el ángulo
            self.angulos_guardados.append((angulo, angulo2))
            self.angles.x = angulo
            self.angles.y = angulo2
            self.publisher_2.publish(self.angles)
            self.posew = ','.join(map(str, row))
            self.execute_subroutine(self.posew)
            
            # Obtener el tiempo de espera dinámico para la fila actual
            wait_time = self.calculate_wait_time(row)
            
            # Esperar el tiempo dinámico antes de la siguiente fila
            time.sleep(wait_time)

        # Reiniciar la bandera de tarea completada
        self.task_completed = False

    def handle_case3(self):
        # matrix de posicion
        self.addr = 0
        self.theta1 = 20
        self.theta2 = self.theta1
        self.vel = 20000
        self.matrixp0 = np.array([self.addr, self.theta1, self.vel, self.addr, self.theta2, self.vel])
        self.angulos_guardados.append((self.theta1, self.theta2))
        self.angles.x = self.theta1
        self.angles.y = self.theta2
        self.publisher_2.publish(self.angles)
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
        self.matrixp1 = np.array([self.addr, self.theta1, self.vel, 0, self.theta2, self.vel])
        self.angulos_guardados.append((-self.theta1, self.theta2))
        self.angles.x = -self.theta1
        self.angles.y = self.theta2
        self.publisher_2.publish(self.angles)
        self.task_completed = False
        self.pose1 = ','.join(map(str, self.matrixp1))
        # Ejecutar subrutina 1
        self.execute_subroutine(self.pose1)
        time.sleep(2)

    def handle_default(self):
        velh = 2000
        angulof1, angulof2 = zip(*self.angulos_guardados)

        # Calcular las direcciones basadas en las sumas de ángulos
        v1 = sum(angulof1)
        v2 = sum(angulof2)
        dir1 = int(v1 > 0)
        dir2 = int(v2 <= 0)
        # print(v1)
        # print(v2)
        # Crear el vector de movimiento
        self.mvec = np.array([dir1, abs(v1), velh, dir2, abs(v2), velh])
        self.angles.x = -v1 if dir else v1
        self.angles.y = v2 if dir2 else -v2
        self.publisher_2.publish(self.angles)
        # Ejecutar la subrutina
        self.home = ','.join(map(str, self.mvec))
        self.execute_subroutine(self.home)
        self.angulos_guardados = []
    
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
        return self.wait

    def timer_callback(self):
        print("Tarea completada")
        # Establecer la bandera de tarea completada
        self.task_completed = True
    
    def listener_callback2(self, msg):
        dato = msg.data
        if dato == "Home":
            pass
        elif dato == "ML":
            self.manual = False
        elif dato == "Manual":
            self.manual = True
        elif dato == "stop":
            exit(0)
    
    def listener_callback3(self, msg):
    
        if self.manual == True:
            dato_a = msg.x
            dato_b = msg.y
            movimiento = np.array([0 if dato_a > 0 else 1, abs(dato_a), 2000, 1 if dato_b > 0 else 0, abs(dato_b), 2000])

            mov = ','.join(map(str, movimiento))
            self.execute_subroutine(mov)
            self.task_completed = False
            self.angles.x = dato_a
            self.angles.y = dato_b
            self.angulos_guardados.append((dato_a, dato_b))
            self.publisher_2.publish(self.angles)
            


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
