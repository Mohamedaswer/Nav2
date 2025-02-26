import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from basic_mobile_robot.action import Delivery  # Import your custom action
from nav2_msgs.action import NavigateToPose  # Import Nav2 action
from geometry_msgs.msg import PoseStamped  # Import PoseStamped for navigation goals

class ButlerActionServer(Node):

    def __init__(self):
        super().__init__('butler_action_server')
        self._action_server = ActionServer(
            self,
            Delivery,
            'order_delivery',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Butler Action Server started...')

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Delivery.Feedback()
        result_msg = Delivery.Result()

        # Get the table number from the goal
        table_number = goal_handle.request.table_number

        # Move to the kitchen
        feedback_msg.current_location = "Kitchen"
        feedback_msg.status = "Moving"
        goal_handle.publish_feedback(feedback_msg)
        kitchen_pose = self.get_kitchen_pose()
        if not await self.navigate_to_pose(kitchen_pose):
            result_msg.success = False
            result_msg.final_status = "Failed to reach Kitchen"
            goal_handle.abort()
            return result_msg

        # Move to the table
        feedback_msg.current_location = f"Table {table_number}"
        feedback_msg.status = "Moving"
        goal_handle.publish_feedback(feedback_msg)
        table_pose = self.get_table_pose(table_number)
        if not await self.navigate_to_pose(table_pose):
            result_msg.success = False
            result_msg.final_status = f"Failed to reach Table {table_number}"
            goal_handle.abort()
            return result_msg

        # Return to home
        feedback_msg.current_location = "Home"
        feedback_msg.status = "Moving"
        goal_handle.publish_feedback(feedback_msg)
        home_pose = self.get_home_pose()
        if not await self.navigate_to_pose(home_pose):
            result_msg.success = False
            result_msg.final_status = "Failed to return Home"
            goal_handle.abort()
            return result_msg

        # Task completed successfully
        result_msg.success = True
        result_msg.final_status = "Completed"
        goal_handle.succeed(result_msg)

    async def navigate_to_pose(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._nav_to_pose_client.wait_for_server()
        send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg)
        await send_goal_future
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected :(')
            return False

        self.get_logger().info('Navigation goal accepted :)')
        result_future = goal_handle.get_result_async()
        await result_future
        return True

    def get_kitchen_pose(self):
        # Define the kitchen pose (replace with actual coordinates)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        pose.pose.orientation.w = 1.0
        return pose

    def get_table_pose(self, table_number):
        # Define the table pose (replace with actual coordinates)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        if table_number == "table1":
            pose.pose.position.x = 3.0
            pose.pose.position.y = 4.0
        elif table_number == "table2":
            pose.pose.position.x = 5.0
            pose.pose.position.y = 6.0
        pose.pose.orientation.w = 1.0
        return pose

    def get_home_pose(self):
        # Define the home pose (replace with actual coordinates)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.orientation.w = 1.0
        return pose

def main(args=None):
    rclpy.init(args=args)
    butler_action_server = ButlerActionServer()
    rclpy.spin(butler_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
