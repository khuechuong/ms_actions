import rclpy
from rclpy.node import Node
from auto_actions.srv import GoToGoal  # Replace with your actual package name

class GoToGoalClient(Node):
    def __init__(self):
        super().__init__('go_to_goal_client')
        self.cli = self.create_client(GoToGoal, 'go_to_goal')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for go_to_goal service...')

    def send_goal(self, x, y):
        req = GoToGoal.Request()
        req.x = x
        req.y = y
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('✅ Goal reached.')
        else:
            self.get_logger().info('❌ Failed to reach goal.')

def main():
    rclpy.init()
    node = GoToGoalClient()
    node.send_goal(2.0, 1.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
