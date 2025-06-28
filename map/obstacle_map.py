# Mueve el robot hacia delante, esquivando obstaculos y manda los datos al nodo
# map.py para que los represente


import numpy as np
import math
import time
import rclpy    #ROS Client Library for the Python language
from rclpy.node import Node     # import the Node module
from rclpy.qos import qos_profile_sensor_data, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy, QoSProfile

#from rclpy.action import ActionClient
from irobot_create_msgs.msg import IrIntensityVector
from irobot_create_msgs.msg import HazardDetectionVector
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import OccupancyGrid, Odometry
from map_msgs.msg import OccupancyGridUpdate


from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter#, ParameterType, ParameterValue


WIDTH = 5
HEIGTH = 5
GRID_SIZE = 0.01         # float


######## NODO  ########

class IRSuscriber(Node):

    def __init__(self, namespace: str = ""):

        super().__init__('node_map')
        # suscription para los sensores IR
        self.subscription_ir = self.create_subscription(
            IrIntensityVector, namespace + '/ir_intensity', self.callback_IR,
            qos_profile_sensor_data)
        self.distance = np.zeros(7)

        # suscription para el bumper
        self.subscription_bump = self.create_subscription(
            HazardDetectionVector, namespace + '/hazard_detection', self.callback_hazard,
            qos_profile_sensor_data)
        
        # publisher para la velocidad del robot
        self.publisher = self.create_publisher(
            Twist, namespace + '/cmd_vel', 10)
        
        # suscription para la posicion
        self.subscription_odom = self.create_subscription(
            Odometry, namespace + '/odom', self.callback_position,
            qos_profile_sensor_data)
        
        self.bumper = -1
        self.freedom = True
        self.timer = 0

        # Coordenadas
        self.posX = 0; self.posY = 0; self.posZ = 0
        self.oriX = 0; self.oriY = 0; self.oriZ = 0; self.oriW = 0


        # ------------------- MAPA -------------------
        qos_policy = QoSProfile(reliability= ReliabilityPolicy.RELIABLE,
        history= HistoryPolicy.KEEP_LAST,
        durability= DurabilityPolicy.TRANSIENT_LOCAL,
        depth= 10)

        # Publisher para generar el mapa
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', qos_policy)
        self.map_update_publisher = self.create_publisher(OccupancyGridUpdate, '/map_updates', qos_policy)

        # self.resolution = 0.01  # m per grid cell (ahora esta como variable global)
        self.n_cells = (int(WIDTH/GRID_SIZE), int(HEIGTH/GRID_SIZE))
        self.map = np.zeros(self.n_cells, dtype=np.int8)
        print(self.map)

        # Cliente para sobreescribir el safety override
        self.cli = self.create_client(SetParameters, '/motion_control/set_parameters')
        self.req = SetParameters.Request()
        self.req.parameters = [Parameter(name='safety_override', value='backup_only').to_parameter_msg()]
        
        print(self.req)
        rclpy.spin_once(self)


    # Callback para los sensores IR
    def callback_IR(self, msg: IrIntensityVector):

        self.freedom = True

        # Para comparar
        sensor_list = ['ir_intensity_side_left', 'ir_intensity_left', 'ir_intensity_front_left',
        'ir_intensity_front_center_left', 'ir_intensity_front_center_right', 'ir_intensity_front_right',
        'ir_intensity_right']

        # Compara sensor_list con frame_id, para asegurarse de que los
        # sensores estan en orden (sino lo estan los ordena usando sensor_list)
        for reading in msg.readings:
            if reading.header.frame_id in sensor_list:
                index = sensor_list.index(reading.header.frame_id)
                self.distance[index] = reading.value
                # Modificado para que no se separe tanto de la pared
                if self.distance[index] > 225 and index in [2, 3, 4]:
                    self.freedom = False
                elif self.distance[index] > 550 and index in [1, 5]:
                    self.freedom = False

        print(f"Array de sensores = [{self.distance}]")
        self.move_robot()


    # Callback para los bumper
    def callback_hazard(self, msg: HazardDetectionVector):

        try:
            bump = msg._detections[0]._header.frame_id
            #print(bump)
            if bump != 'base_link':
                self.timer = time.time_ns()
                if bump == 'bump_right' or bump == 'bump_front_right':
                    self.bumper = 1
                else:
                    self.bumper = 0
            else:
                # print("False positive - (Base Link)")
                self.bumper = -1
        except:
            self.bumper = self.bumper
            # print("False positive")

    # Callback para la odometria
    def callback_position(self, msg: Odometry):

        self.posX = msg._pose.pose.position.x
        self.posY = msg._pose.pose.position.y
        self.posZ = msg._pose.pose.position.z

        self.oriX = msg._pose.pose.orientation.x
        self.oriY = msg._pose.pose.orientation.y
        self.oriZ = msg._pose.pose.orientation.z
        self.oriW = msg._pose.pose.orientation.w

        #print(f"Position: x: {self.posX}, y: {self.posY}, z: {self.posZ}")
        #print(f"Orientation: x: {self.oriX}, y: {self.oriY}, z: {self.oriZ}, w: {self.oriW}")

        # Aqui deberia meter algo para que el mapa se actualice  <----
        self.update_map()

    
    def quaternion2euler(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)

        REVISAR CUATERNIONES (es basicamente una matriz)

        """
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


    def update_map(self):
        
        # Coordenadas a actualizar (esta en cuaterniones) REVISAR
        print("Actualizando el mapa")

        for i in range(7):
            if self.distance[i] > 200:
                angle_offset = [-65.3, -38, -20, -3.0, 14.25, 34.0, 65.3][i] * math.pi / 180
                roll, pitch, yaw = self.quaternion2euler(self.oriX, self.oriY, self.oriZ, self.oriW)  # El roll y el pitch no se usan
                alpha = yaw - angle_offset
                d=8/100
                x_map = (self.posX + d*np.cos(alpha))  / GRID_SIZE
                y_map = (self.posY + d*np.sin(alpha))  / GRID_SIZE

                x_map = int(x_map + WIDTH/GRID_SIZE/2)
                y_map = int(-y_map + HEIGTH/GRID_SIZE/2)
                print(f"x_map = {x_map}, y_map = {y_map}")
                self.map[x_map, y_map] = 100
                
        self.publish_map()


    # Funcion para el movimiento del robot
    def move_robot(self):

        self.twist = Twist()
        
        self.twist.linear.x  = 0.0
        self.twist.linear.y  = 0.0
        self.twist.linear.z  = 0.0

        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

        if self.bumper == -1:
            if self.freedom:
                # Move forward
                self.twist.linear.x = 0.2     #speed

            else:
                # Turn
                direction = 0
                for i in range(7):
                    if self.distance[i] > self.distance[direction]:    # Coje el valor más alto de los 7
                        direction = i
                if direction > 4:                # Obstaculo a la derecha
                    self.twist.angular.z = 0.4
                else:                            # Obstaculo a la izquierda
                    self.twist.angular.z = -0.4
                     
        else: 
            self.twist.linear.x = -0.15
            if self.bumper == 1:
                self.twist.angular.z = 0.4
            else:
                self.twist.angular.z = -0.4

            #print(time.time_ns() - self.timer)
            if (time.time_ns() - self.timer) > 40e7:
                self.bumper = -1
                
        
        ################### BORRAR ######################
        # self.twist.linear.x  = 0.0
        # self.twist.linear.y  = 0.0
        # self.twist.linear.z  = 0.0

        # self.twist.angular.x = 0.0
        # self.twist.angular.y = 0.0
        # self.twist.angular.z = 0.0
        ################### BORRAR ######################

        self.publisher.publish(self.twist)


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

            # grid_msg = OccupancyGridUpdate()
            # # Header
            # grid_msg.header.stamp = self.get_clock().now().to_msg()
            # grid_msg.header.frame_id = 'map_updates'
            # grid_msg.data = self.map.flatten().tolist()
            # # print(grid_msg.data)
            # grid_msg.width = self.n_cells[0]
            # grid_msg.height = self.n_cells[1]
            # grid_msg.x = int(-self.n_cells[0] * GRID_SIZE / 2)
            # grid_msg.y = int(-self.n_cells[1] * GRID_SIZE / 2)
            # self.map_update_publisher.publish(grid_msg)



    # Para imprimir el valor de los sensores IR (outdated)
    # def receive_data(self):
        
    #     return self.distance
    

###############################################################

def main(args=None):
    rclpy.init(args=args)
    ir_subscriber = IRSuscriber()
    try:
        while True:
                rclpy.spin(ir_subscriber)
                #distancia = ir_subscriber.receive_data()
                #print(f"Array de sensores = [{distancia}]")

    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print("Done")
        ir_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


