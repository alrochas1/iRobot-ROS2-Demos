# Simula el nodo (que deberia existir, pero por algun motivo no funciona) encargado de dibujar el mapa.
# Esta suscrito al topic de Odometria y al del mapa y pinta los datos recibidos

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import matplotlib.pyplot as plt

class MapSubscriber(Node):

    def __init__(self):
        super().__init__('map_subscriber')
        self.subscription = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odometry_callback,
            qos_profile_sensor_data)

        self.fig, self.ax = plt.subplots()
        self.width = 0; self.height = 0; self.res = 0
        self.odom = []
        

    def map_callback(self, msg):
        self.width = int (msg.info.width)
        self.height = int (msg.info.height)
        data = np.array(msg.data).reshape((self.height, self.width))
        self.res = msg.info.resolution

        self.ax.clear()
        self.ax.imshow(data, cmap='gray', origin='lower')
        if len(self.odom) > 2:
            odom_x, odom_y = zip(*self.odom)
            # print(f"pos_x: {odom_x}, pos_y: {odom_y}")
            self.ax.plot(odom_y, odom_x, 'r.', markersize=1)
        self.ax.set_title('Occupancy Grid Map')
        self.ax.set_xlabel('y (cells)')
        self.ax.set_ylabel('x (cells)')
        self.fig.canvas.draw()

    def odometry_callback(self, msg):
        
        if self.res != 0:   # Si ya ha llamado a map_callback
            # ------- CORREGIR -------
            posX = (msg._pose.pose.position.x/self.res + self.width/2)
            posY = (-msg._pose.pose.position.y/self.res + self.height/2)
            self.odom.append((posX, posY))



def main(args=None):
    rclpy.init(args=args)
    map_subscriber = MapSubscriber()

    plt.ion()
    try:
        while rclpy.ok():
            rclpy.spin_once(map_subscriber)
            plt.pause(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        map_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
