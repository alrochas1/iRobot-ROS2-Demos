import rclpy    #ROS Client Library for the Python language
from rclpy.node import Node     # import the Node module
from rclpy.action import ActionClient

from irobot_create_msgs.action import RotateAngle


class Rotate_Client(Node):

    def __init__(self):
    
        super().__init__('rotate_client')
        self._action_client = ActionClient(self, RotateAngle, '/rotate_angle')


    def send_goal(self, angle=3.14, max_rotation_speed=0.5):
        

        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle
        goal_msg.max_rotation_speed = max_rotation_speed
        print(goal_msg)

        if not self._action_client.wait_for_server(5):
            print("Time reached limit")
        

        return self._action_client.send_goal_async(goal_msg)



def main(args=None):
    rclpy.init(args=args)

    rotate_client = Rotate_Client()
    future = rotate_client.send_goal()
    #print(future)

    # The future is completed when an action server accepts or rejects the goal.
    print("spin")
    rclpy.spin_until_future_complete(rotate_client, future)
    rclpy.shutdown()


if __name__ == '__main__':
    main()