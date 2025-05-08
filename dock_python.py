# Muy poco eficiente

import rclpy    #ROS Client Library for the Python language
from rclpy.node import Node     # import the Node module
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from irobot_create_msgs.action import Dock
from irobot_create_msgs.action import RotateAngle
from irobot_create_msgs.msg import DockStatus

    
    
class Docking_Action_Client(Node):

    def __init__(self,  namespace: str = ""):
    
        super().__init__('dock_client')
        self.subscription = self.create_subscription(
            DockStatus, namespace + '/dock_status', self.listener_callback,
            qos_profile_sensor_data)
        self.docked = False
        

    def listener_callback(self, msg: DockStatus):

        print(f"The robot can see the docker: {msg.dock_visible}")
        print(f"The robot is docked: {msg.is_docked}")

        self.docked = msg.is_docked
        self.send_goal(msg.dock_visible)

        # if msg.dock_visible:
            
        #     self._action_client = ActionClient(self, Dock, 'Dock')
        #     self.send_goal_dock()
        # else:
            
        #     self._action_client = ActionClient(self, RotateAngle, 'Rotate')
        #     self.send_goal_rotate()
            

    def send_goal(self, visible):

        if not (self.docked):
            if visible:
                
                self._action_client = ActionClient(self, Dock, 'dock')
                print("Docking ...")
                goal_msg = Dock.Goal()

            else:
                
                self._action_client = ActionClient(self, RotateAngle, '/rotate_angle')

                print("Rotating ... ")
                goal_msg = RotateAngle.Goal()
                goal_msg.angle = 3.14
                goal_msg.max_rotation_speed = 0.5
            
            print(goal_msg)
            self._action_client.wait_for_server()
            return self._action_client.send_goal_async(goal_msg)

        else:
            print("Docked")
            return None
        
    
    
    # Seguro que hay una manera mejor de hacer esto 
    def robot_docked(self):

        return self.docked

    

#######################################

def main(args=None):
    rclpy.init(args=args)

    dock_client = Docking_Action_Client()
    state = dock_client.robot_docked()
    try:
        while state == False:
            rclpy.spin_once(dock_client)
            state = dock_client.robot_docked()
            #print(f"El robot esta en el docker: {state}")
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print("Done")
        dock_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()