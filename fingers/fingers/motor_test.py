# Simple class for calling action server and testing motor operations


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from hand_msgs.action import Gripper


class MotorTestClient(Node):

    def __init__(self):
        super().__init__('motor_test_client')
        self._action_client = ActionClient(self, Gripper, 'gripper_command')
        self._action_client.wait_for_server()

    def send_goal(self):
        goal_msg = Gripper.Goal()
        goal_msg.command = "close-parallel"

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def goal_response_callback(self, future):
        goal_handle = future.result()
        print(goal_handle)
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    action_client = MotorTestClient()

    future = action_client.send_goal()

    rclpy.spin(action_client)
    print("Done")


if __name__ == '__main__':
    main()