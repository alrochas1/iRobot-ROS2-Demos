import numpy as np
import rclpy    #ROS Client Library for the Python language
from rclpy.node import Node     # import the Node module
from rclpy.qos import qos_profile_sensor_data

from irobot_create_msgs.msg import IrIntensityVector


class IRSuscriber(Node):

    def __init__(self, namespace: str = ""):

        super().__init__('ir_intensity')
        self.subscription = self.create_subscription(
            IrIntensityVector, namespace + '/ir_intensity', self.listener_callback,
            qos_profile_sensor_data)
        
        self.distance = np.zeros(7)

    def listener_callback(self, msg: IrIntensityVector):
        
        #self.get_logger().info('I heard: "%s"' % msg)
        self.update_distance(msg)

    def update_distance(self, msg):
        
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


    def receive_data(self):
        
        return self.distance
    

def main(args=None):
    rclpy.init(args=args)
    ir_subscriber = IRSuscriber()
    try:
        while True:
                rclpy.spin_once(ir_subscriber)
                distancia = ir_subscriber.receive_data()
                print(f"Array de sensores = [{distancia}]")

    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print("Done")
        ir_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()