#! /usr/bin/env python3

import tkinter as tk
from tkinter import ttk
from tkinter import StringVar
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sv_ttk
from geometry_msgs.msg import Twist, Point
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class ROSNode(Node):
    def __init__(self):
        super().__init__("control")
        # Configuración de suscripciones y temporizador
        self.subscription = self.create_subscription(String, 
                                                     '/movement', 
                                                     self.listener_callback, 
                                                     10)
        self.subscription2 = self.create_subscription(Twist, 
                                                      '/command', 
                                                      self.listener_callback2, 
                                                      10)
        self.subscription3 = self.create_subscription(Point, 
                                                      '/angles', 
                                                      self.listener_callback3, 
                                                      10)
        self.publisher_angle = self.create_publisher(Point, 
                                                 '/slider', 
                                                 10)
        self.publisher_1 = self.create_publisher(String, 
                                                 '/mode', 
                                                 10)
        self.move_value = " "
        self.angle1 = 0.0
        self.angle2 = 0.0

    def listener_callback(self, msg):
        # Ahora, self.move_value es una variable de la clase GUIApp
        self.move_value = msg.data

    def listener_callback2(self, msg):
        self.vel1 = msg.linear.x
        self.angle1 = msg.linear.y
    
    def listener_callback3(self, msg):
        self.angle = msg.x
        self.angle = msg.y
    
    def publish_mode(self, mode):
        msg = String()
        msg.data = mode
        self.publisher_1.publish(msg)
    
    def publish_angle(self, slider1, slider2):
        msg = Point()
        msg.x = float(slider1)
        msg.y = float(slider2)
        self.publisher_angle.publish(msg)
        


class GUIApp:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.root = tk.Tk()
        self.root.title("ROS2 EXO")
        self.root.geometry("800x600")  # Relación de aspecto 1:1
        sv_ttk.set_theme("dark")
        # Configurar Matplotlib
        plt.style.use('dark_background')  # Fondo negro
        self.fig, self.ax1 = plt.subplots(1, 1, figsize=(6, 4))  # Solo una subgráfica
        plt.title('DOF 2 Exoesqeleto')
        plt.grid()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=1)

        # Configurar botón de publicar mensaje
        self.button_publish = ttk.Button(self.root, text="Publish Angles", command=self.publish_message, style='TButton')
        self.button_publish.pack(side=tk.LEFT, padx=(10, 10), pady=10)

        # Configurar botones adicionales en la misma fila
        self.button_automatico = ttk.Button(self.root, text="Automatic", command=self.click_automatico, style='TButton')
        self.button_automatico.pack(side=tk.LEFT, padx=(10, 10), pady=10)

        self.button_ml = ttk.Button(self.root, text="ML", command=self.click_ml, style='TButton')
        self.button_ml.pack(side=tk.LEFT, padx=(10, 10), pady=10)

        self.button_manual = ttk.Button(self.root, text="Manual", command=self.click_manual, style='TButton')
        self.button_manual.pack(side=tk.LEFT, padx=(10, 10), pady=10)

        # Configurar botón "Stop" redondo
        self.button_stop = tk.Button(self.root, text="Stop", command=self.click_stop, bg='red', fg='white', padx=15, pady=10,
                                     font=('Arial', 14), relief=tk.GROOVE, bd=0)
        self.button_stop.pack(side=tk.LEFT, padx=(10, 10), pady=10)

        # Configurar recuadro de información
        self.info_frame = tk.Frame(self.root, bg='gray', padx=10, pady=10)
        self.info_frame.pack(side=tk.LEFT, fill=tk.Y)
        
        self.info_label = tk.Label(self.info_frame, text = "Status: " + self.ros_node.move_value, font=('Arial', 12), bg='gray', fg='white', justify=tk.CENTER)
        self.info_label.pack()

        # Configurar sliders
        self.slider_frame = tk.Frame(self.root, padx=10, pady=10)
        self.slider_frame.pack(side=tk.LEFT, fill=tk.Y)

        self.slider1_label = tk.Label(self.slider_frame, text="Joint 1", font=('Arial', 12))
        self.slider1_label.pack()

        self.slider1 = tk.Scale(self.slider_frame, from_=-20, to=20, orient=tk.HORIZONTAL, length=200)
        self.slider1.pack()

        self.slider2_label = tk.Label(self.slider_frame, text="Joint 2", font=('Arial', 12))
        self.slider2_label.pack()

        self.slider2 = tk.Scale(self.slider_frame, from_=-50, to=50, orient=tk.HORIZONTAL, length=200)
        self.slider2.pack()

    def update_label_from_node(self):
        dato = self.ros_node.move_value
        if dato == "walk":
            self.info_label.config(text="Status: " + dato, bg="blue", justify=tk.CENTER)
        elif dato == "march":
            self.info_label.config(text="Status: " + dato, bg="red", justify=tk.CENTER)
        elif dato == "pose1":
            self.info_label.config(text="Status: " + dato, bg="magenta", justify=tk.CENTER)
        elif dato == "pose2":
            self.info_label.config(text="Status: " + dato, bg="purple", justify=tk.CENTER)
        elif dato == "stay":
            self.info_label.config(text="Status: " + dato, bg="green", justify=tk.CENTER)
        self.root.update_idletasks()

    def publish_message(self):
        value_slider1 = self.slider1.get()
        value_slider2 = self.slider2.get()
        self.ros_node.publish_angle(value_slider1, value_slider2)

    def click_automatico(self):
        message = "Automatico"
        self.ros_node.publish_mode(message)

    def click_ml(self):
        message = "ML"
        self.ros_node.publish_mode(message)

    def click_manual(self):
        message = "Manual"
        self.ros_node.publish_mode(message)

    def click_stop(self):
        message = "stop"
        self.ros_node.publish_mode(message)
        
    def update_plots(self):
        pass

    def update_from_ros(self):
        # Actualizar gráficos desde ROS cada segundo
        self.update_label_from_node()
        self.update_plots()
        self.root.after(1000, self.update_from_ros)


    def run(self):
        self.root.after(1000, self.update_from_ros)  # Iniciar actualizaciones desde ROS
        self.root.mainloop()

def main(args = None):
    rclpy.init(args=args)
    
    ros_node = ROSNode()

    # Hilo para ROS
    ros_thread_instance = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread_instance.start()

    # Interfaz gráfica
    gui_app = GUIApp(ros_node)
    gui_app.run()

    ros_thread_instance.join()  # Esperar al hilo de ROS

    rclpy.shutdown()

if __name__ == '__main__':
    main()