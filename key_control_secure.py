import rclpy    #ROS Client Library for the Python language
from rclpy.node import Node     # import the Node module


from irobot_create_msgs.msg import IrIntensityVector
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, Vector3  

from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter#, ParameterType, ParameterValue

import pynput.keyboard
import threading
import numpy as np



################ KEYBOARD ################

class KeypressCounter(threading.Thread):
    
    def __init__(self):

        super().__init__()
        self.key_p = -1

    def run(self):
        listener = pynput.keyboard.Listener(on_press=self.on_press,daemon=False)
        listener.start()
    def on_press(self, key):
        
        if key in [pynput.keyboard.Key.up]:
            self.key_p = 1

        elif key in [pynput.keyboard.Key.right]:
            self.key_p = 2
        
        elif key in [pynput.keyboard.Key.left]:
            self.key_p = 3

        elif key in [pynput.keyboard.Key.down]:
            self.key_p = 4

        elif key in [pynput.keyboard.Key.space]:
            self.key_p = 0
    


################ ROBOT ################

class robotMovement(Node):
    
    def __init__(self, namespace: str = ""):

        super().__init__('robotMovement')

        # Cliente para sobreescribir el safety override
        self.cli = self.create_client(SetParameters, '/motion_control/set_parameters')
        self.req = SetParameters.Request()
        self.req.parameters = [Parameter(name='safety_override', value='backup_only').to_parameter_msg()]
        
        print(self.req)
        


        self.subscription = self.create_subscription(
            IrIntensityVector, namespace + '/ir_intensity', self.listener_callback,
            qos_profile_sensor_data)
        self.publisher = self.create_publisher(
            Twist, namespace + '/cmd_vel', 10)

        self.key_p = -1     # Si, es una cutrez
        rclpy.spin_once(self)


        
    # ---------------------------------------------
    def listener_callback(self, msg: IrIntensityVector):

        self.update_distance(msg)

    def update_distance(self, msg):
        
        self.distance = np.zeros(7)
        self.freedom = True
        for i in range(7):
            self.distance[i] = msg.readings[i].value
            if self.distance[i] > 200:
                self.freedom = False
            i = i+1
        print(f"Array de sensores = [{self.distance}]")

    

    def move_robot(self, key_p):

        self.twist = Twist()
        
        self.twist.linear.x  = 0.0
        self.twist.linear.y  = 0.0
        self.twist.linear.z  = 0.0

        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

        # 0 -> stop, 1 -> forward, 2,3 -> turn, 4-> back
        if key_p == 0:
                self.twist.linear.x = 0.0     #stop

        elif key_p == 2:
           self.twist.angular.z = -0.8     
        
        elif key_p == 3:
            self.twist.angular.z = 0.8
        
        elif self.freedom:
            if key_p == 1:
                self.twist.linear.x = 0.2     #speed
            
            elif key_p == 4:
                self.twist.linear.x = -0.2     #speed

        self.publisher.publish(self.twist)



################ MAIN ################

def main(args=None):

    rclpy.init(args=args)

    keypress = KeypressCounter()
    keypress.start()

    movement_client = robotMovement()
    try:
        while True:
            rclpy.spin_once(movement_client)    # Con esto recibe la info
            movement_client.move_robot(keypress.key_p)  # Con esto actualiza el movimiento
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print("Done")
        robotMovement.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()