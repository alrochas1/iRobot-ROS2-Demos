# Esta version tiene en cuenta los obstaculos y los dibuja en un mapa

import numpy as np
import math
import time

import rclpy    # ROS Client Library for the Python language (se puede quitar)
from rclpy.node import Node     # import the Node module
import rclpy.node
from rclpy.qos import qos_profile_sensor_data, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry, OccupancyGrid
from irobot_create_msgs.msg import IrIntensityVector
from geometry_msgs.msg import Twist

# Variables del mapa
WIDTH = 5
HEIGTH = 5
GRID_SIZE = 0.01         # float

class Waypoint_Move(Node):
     
    def __init__(self):
          
        super().__init__('waypoint')
        
        # Suscripcion a la odometria
        self.odom_suscriber = self.create_subscription(Odometry, '/odom', self.update_position,
        qos_profile_sensor_data)

        # Suscripcion a los sensores IR
        self.ir_suscriber = self.create_subscription(IrIntensityVector, '/ir_intensity', self.update_distance,
        qos_profile_sensor_data)
        self.distance = np.zeros(7)
        self.state = "free"
        self.timer = None

        # Publisher para la velocidad
        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)
        
        # Coordenadas
        self.posX = float('inf'); self.posY = float('inf')    # Evita problemas al inicializar 
        self.yaw = 0
        self.epsilon = 0.005 # Error en la posisicion final


        # Publisher para generar el mapa
        qos_policy = QoSProfile(reliability= ReliabilityPolicy.RELIABLE,
        history= HistoryPolicy.KEEP_LAST,
        durability= DurabilityPolicy.TRANSIENT_LOCAL,
        depth= 10)

        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', qos_policy)
        self.n_cells = (int(WIDTH/GRID_SIZE), int(HEIGTH/GRID_SIZE))
        self.map = np.zeros(self.n_cells, dtype=np.int8)
        
        

    # Actualiza las coordenadas del robot
    def update_position(self, msg: Odometry):
     
        self.posX = msg._pose.pose.position.x
        self.posY = msg._pose.pose.position.y
        self.posZ = msg._pose.pose.position.z

        # Solo hace falta el yaw (quien usa cuaterniones en 2D ????)
        roll, pitch, self.yaw = self.quaternion2euler(msg._pose.pose.orientation.x,
        msg._pose.pose.orientation.y, msg._pose.pose.orientation.z, msg._pose.pose.orientation.w)

        self.update_map()


    def quaternion2euler(self, x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        
        return roll, pitch, yaw  # in radians
    

    # Actualiza el valor de los sensores IR
    def update_distance(self, msg: IrIntensityVector):

        # Para comparar
        sensor_list = ['ir_intensity_side_left', 'ir_intensity_left', 'ir_intensity_front_left',
        'ir_intensity_front_center_left', 'ir_intensity_front_center_right', 'ir_intensity_front_right',
        'ir_intensity_right']


        # Compara sensor_list con frame_id, para asegurarse de que los
        # sensores estan en orden (sino lo estan los ordena usando sensor_list)
        for reading in msg.readings:
            if reading.header.frame_id in sensor_list:
                index = sensor_list.index(reading.header.frame_id)
                d = reading.value
                self.distance[index] = -13.9813 + (33.00164 + 13.9813) / (1 + (d / 169.7088) ** 0.3270058)
                # self.distance[index] = self.distance[index]/100     # En cm
                # Modificado para que no se separe tanto de la pared
                if self.distance[index] < 9 and index in [2, 3, 4]:
                    self.state = "obstacle"
                elif self.distance[index] < 5 and index in [1, 5]:
                    self.state = "obstacle"
                elif self.distance[index] < 3.5 and index in [0, 6]:
                    self.state = "obstacle"


        # # Estos no hacen falta en teoria
        # self.distance[3] = 0
        # self.distance[2] = 0

        print(f"Array de sensores = [{self.distance}]")



    # Actualiza los datos del mapa
    def update_map(self):
        
        # Coordenadas a actualizar (esta en cuaterniones) REVISAR
        print("Actualizando el mapa")

        for i in range(7):
            if self.distance[i] < 9:
                angle_offset = [-65.3, -38, -20, -3.0, 14.25, 34.0, 65.3][i] * math.pi / 180
                alpha = self.yaw - angle_offset
                # Ajuste para calcular la distancia
                # d = -13.9813 + (33.00164 + 13.9813) / (1 + (self.distance[i] / 169.7088) ** 0.3270058)
                d = self.distance[i]/100
                x_map = (self.posX + d*np.cos(alpha))  / GRID_SIZE
                y_map = (self.posY + d*np.sin(alpha))  / GRID_SIZE

                x_map = int(x_map + WIDTH/GRID_SIZE/2)
                y_map = int(-y_map + HEIGTH/GRID_SIZE/2)
                print(f"x_map = {x_map}, y_map = {y_map}")
                self.map[x_map, y_map] = 100
                
        self.publish_map()



    # Decide como mover el robot
    def move_robot(self, waypoint):

        twist = Twist()
        print(self.state)

        if self.state == "obstacle":
            # Si hay obstaculos se mueve para esquivarlos
            self.avoid_obstacle(twist)
            waypoint_reached = False
        else:
            # Si no hay obstaculos avanza a waypoint
            waypoint_reached = self.move2waypoint(waypoint, twist)

        self.publisher.publish(twist)
        return waypoint_reached


    # Mueve hacia el waypoint
    def move2waypoint(self, waypoint, twist):

        # Si la posicion esta dentro del rango aceptado
        if self.epsilon > abs(self.posX - waypoint[0]) and self.epsilon > abs(self.posY - waypoint[1]):
            error = np.max([abs(self.posX - waypoint[0]), abs(self.posY - waypoint[1])])
            print(f'Esta en el waypoint {waypoint}, con un error aproximado de {error}')
            return True

        # Si no esta dentro del rango
        else:
            # Calcula la direccion del waypoint
            alpha = math.atan2(waypoint[1] - self.posY, waypoint[0] - self.posX)
            diff_angle = alpha - self.yaw # Para evitar problemas al girar
            diff_angle = (diff_angle + math.pi) % (2 * math.pi) - math.pi


            # Move Forward (si esta mirando hacia al waypoint)
            if abs(diff_angle) < self.epsilon*20:
                print("Move Forward")
                twist.linear.x  = 0.15
            
            # Turn (gira hacia el waypoint)
            else:
                print("Turning ...")
                twist.angular.z = 0.65 if diff_angle > 0 else -0.65
                
            return False

    

    # Esquiva el obstaculo
    def avoid_obstacle(self, twist):

        max_distance = np.min(self.distance) 
        if max_distance < 10:

            direction = np.argmin(self.distance)
            if direction in [2, 3, 4]:
                twist.angular.z = 0.5
            elif direction in [1, 5]:
                twist.angular.z = 0.3
            else:
                if np.min(self.distance) < 4:
                    twist.angular.z = 0.2
                    twist.linear.x = 0.05
                else:
                    D = [d for d in self.distance if d != np.min(self.distance)]
                    if np.min(D) > 10:
                        twist.linear.x = 0.25
                    else:
                        twist.angular.z = 0.1 

            twist.angular.z = -twist.angular.z if direction < 4 else twist.angular.z

        else:
            self.state = "free"



        # # Esta un poco confuso porque es un apaño de lo que estaba antes
        # if np.max(self.distance) > 225:
        #     direction = np.argmax(self.distance)
        #     if direction in [2, 3, 4] or np.max(self.distance) > 550:
        #         twist.angular.z = 0.75 if direction > 3 else -0.75
        #     else:
        #         twist.angular.z = 0.15 if direction > 3 else -0.15

        # else:
        #     twist.linear.x  = 0.15
        #     if self.timer is None:
        #         self.timer = time.time_ns()
            
        #     if (time.time_ns() - self.timer) > 60e7:
        #         self.timer = None
        #         self.state = "free"


    # Publica la informacion del mapa en el topic /map
    def publish_map(self):
            grid_msg = OccupancyGrid()
            # Header
            grid_msg.header.stamp = self.get_clock().now().to_msg()
            grid_msg.header.frame_id = 'map'
            # Info
            grid_msg.info.resolution = GRID_SIZE
            grid_msg.info.width = self.n_cells[0]
            grid_msg.info.height = self.n_cells[1]
            # Coordenadas de la primera celda (definido para que el 0,0 quede en el centro del mapa)
            grid_msg.info.origin.position.x = -WIDTH * GRID_SIZE / 2
            grid_msg.info.origin.position.y = -HEIGTH * GRID_SIZE / 2
            grid_msg.info.origin.position.z = 0.0
            grid_msg.data = self.map.flatten().tolist()
            #print(grid_msg)
            self.map_publisher.publish(grid_msg)

