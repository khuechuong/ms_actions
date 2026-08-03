import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import quaternion_from_euler

from xarm_msgs.srv import PlanPose, PlanExec
from auto_actions.srv import Inspect

class InspectClient(Node):
    def __init__(self):
        super().__init__("inspect_client")
        # Create service clients
        self.plan_cli = self.create_client(Inspect, 'inspect')

        # Wait until both services are ready
        self.get_logger().info('Waiting for /inspect services...')
        if not self.plan_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/inspect service not available')
            rclpy.shutdown(); return
        self.get_logger().info('Services available, starting sequence')

    def send_goal(self, goal):
        plan_req = Inspect.Request()
        plan_req.robot_pose = Point()
        plan_req.robot_pose.x = goal[0]
        plan_req.robot_pose.y = goal[1]
        plan_req.robot_pose.z = goal[2]


        plan_req.inspect_area = Point()
        plan_req.inspect_area.x = goal[3]
        plan_req.inspect_area.y = goal[4]
        plan_req.inspect_area.z = goal[5]
        future_plan = self.plan_cli.call_async(plan_req)
        rclpy.spin_until_future_complete(self, future_plan)
        if not future_plan.result().success:
            self.get_logger().info('❌ Failed to plan goal.')
            return

def main():
    rclpy.init()
    node = InspectClient()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    node.send_goal([0.0, 0.0, 0.0, 0.1, -0.8, 0.39]) # RIGHT
    rclpy.shutdown()

if __name__ == '__main__':
    main()