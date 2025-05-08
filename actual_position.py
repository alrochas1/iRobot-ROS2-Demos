# Para ver la position mas comodamente (porque /odom es un poco incomodo)

import sys
import rclpy    #ROS Client Library for the Python language
from rclpy.node import Node     # import the Node module
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

import math


class PositionSubscriber(Node):
    '''
    An example of subscribing to a ROS2 topic.
    A Node listening to the /battery_state topic.
    '''

    def __init__(self, namespace: str = ""):
        '''
        Purpose
        -------
        initialized by calling the Node constructor, naming our node 
        '''
        super().__init__('actual_position')
        self.subscription = self.create_subscription(
            Odometry, namespace + '/odom', self.listener_callback,
            qos_profile_sensor_data)


    def quaternion2euler(self, x, y, z, w):
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        
        return yaw  # in radians


    def listener_callback(self, msg: Odometry):
        '''
        Purpose
        -------
        Whenever our subscriber (listener) get's a message this function is 
        'called back' to and ran.
        '''
        #self.get_logger().info('I heard: "%s"' % msg)
        self.printPosition(msg)

    def printPosition(self, msg):
        '''
        :type msg: Odometry
        :rtype: None

        An example of how to get components of the msg returned from a topic.
        '''
        # We can get components of the message by using the '.' dot operator
        x = msg._pose.pose.position.x
        y = msg._pose.pose.position.y

        yaw = self.quaternion2euler(msg._pose.pose.orientation.x, msg._pose.pose.orientation.y, 
        msg._pose.pose.orientation.z, msg._pose.pose.orientation.w)

        #z = msg._pose.pose.position.z
        print(f"Position: x: {x}, y: {y}, yaw: {yaw}")


def main(args=None):
    rclpy.init(args=args)

    position_subscriber = PositionSubscriber()
    try:
        rclpy.spin(position_subscriber)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print("Done")
        position_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()