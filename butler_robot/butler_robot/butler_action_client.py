import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from basic_mobile_robot.action import Delivery  # Import your custom action

class ButlerActionClient(Node):

    def __init__(self):
        super().__init__('butler_action_client')
        self._action_client = ActionClient(self, Delivery, 'order_delivery')
        self.get_logger().info('Butler Action Client started...')

    def send_goal(self, table_number, require_confirmation=False, allow_cancellation=False, multiple_orders=False, order_list=[]):
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create a goal message
        goal_msg = Delivery.Goal()
        goal_msg.table_number = table_number
        goal_msg.require_confirmation = require_confirmation
        goal_msg.allow_cancellation = allow_cancellation
        goal_msg.multiple_orders = multiple_orders
        goal_msg.order_list = order_list

        # Send the goal to the action server
        self.get_logger().info('Sending goal...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Register a callback for when the goal is accepted
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Register a callback for when the result is available
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: Current Location = {feedback.current_location}, Status = {feedback.status}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Final Result: Success = {result.success}, Final Status = {result.final_status}')
        rclpy.shutdown()

def get_user_input():
    # Prompt the user for input
    table_number = input("Enter the table number: ")

    require_confirmation = input("Require confirmation? (yes/no): ").strip().lower() == "yes"
    allow_cancellation = input("Allow cancellation? (yes/no): ").strip().lower() == "yes"
    multiple_orders = input("Handle multiple orders? (yes/no): ").strip().lower() == "yes"

    order_list = []
    if multiple_orders:
        print("Enter the list of tables for multiple orders (comma-separated):")
        order_list = input().strip().split(',')

    return table_number, require_confirmation, allow_cancellation, multiple_orders, order_list

def main(args=None):
    rclpy.init(args=args)

    # Get user input
    table_number, require_confirmation, allow_cancellation, multiple_orders, order_list = get_user_input()

    # Create the action client
    action_client = ButlerActionClient()

    # Send a goal to the action server
    action_client.send_goal(table_number, require_confirmation, allow_cancellation, multiple_orders, order_list)

    # Spin the node to process callbacks
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
