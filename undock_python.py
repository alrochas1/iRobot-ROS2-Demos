import rclpy    #ROS Client Library for the Python language
from rclpy.node import Node     # import the Node module
from rclpy.action import ActionClient

from irobot_create_msgs.action import Undock
from irobot_create_msgs.srv import ResetPose


class Undocking_Action_Client(Node):

    def __init__(self):
    
        super().__init__('undocking_action_client')
        self._action_client = ActionClient(self, Undock, 'undock')

        # Cliente para que siempre empiece en el 0
        self.cli = self.create_client(ResetPose, '/reset_pose')

        self.req = ResetPose.Request()
        rclpy.spin_once(self)


    def send_goal(self):

        goal_msg = Undock.Goal()
        print(goal_msg)

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)



def main(args=None):
    rclpy.init(args=args)

    undock_client = Undocking_Action_Client()
    future = undock_client.send_goal()

    # The future is completed when an action server accepts or rejects the goal.
    rclpy.spin_until_future_complete(undock_client, future)
    rclpy.shutdown()


if __name__ == '__main__':
    main()