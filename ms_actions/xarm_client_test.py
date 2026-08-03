import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import quaternion_from_euler

from xarm_msgs.srv import PlanPose, PlanExec

class xArmClient(Node):
    def __init__(self):
        super().__init__("xarm_client")
        # Create service clients
        self.plan_cli = self.create_client(PlanPose, '/xarm_pose_plan')
        self.exec_cli = self.create_client(PlanExec, '/xarm_exec_plan')

        # Wait until both services are ready
        self.get_logger().info('Waiting for xarm services...')
        if not self.plan_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('PlanPose service not available')
            rclpy.shutdown(); return
        if not self.exec_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('PlanExec service not available')
            rclpy.shutdown(); return
        self.get_logger().info('Services available, starting sequence')

    def send_goal(self, goal):
        plan_req = PlanPose.Request()
        plan_req.target = Pose()
        plan_req.target.position.x = goal[0]
        plan_req.target.position.y = goal[1]
        plan_req.target.position.z = goal[2]
        # --- Orientation ---
        if len(goal) == 6:
            # Input is RPY (roll, pitch, yaw)
            roll, pitch, yaw = goal[3:6]
            qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        elif len(goal) == 7:
            # Input is Quaternion (x, y, z, w)
            qx, qy, qz, qw = goal[3:7]
        else:
            raise ValueError("Goal must have 6 (rpy) or 7 (quaternion) elements.")
        plan_req.target.orientation.x = qx
        plan_req.target.orientation.y = qy
        plan_req.target.orientation.z = qz
        plan_req.target.orientation.w = qw
        future_plan = self.plan_cli.call_async(plan_req)
        rclpy.spin_until_future_complete(self, future_plan)
        if not future_plan.result().success:
            self.get_logger().info('❌ Failed to plan goal.')
            return
        # Execute
        self.get_logger().info('✅ Plan successful, executing trajectory...')
        exec_req = PlanExec.Request()
        exec_req.wait = True  # wait until motion completes
        future_exec = self.exec_cli.call_async(exec_req)
        rclpy.spin_until_future_complete(self, future_exec)
        try:
            if future_exec.result().success:
                self.get_logger().info('✅ Execution completed successfully.')
            else:
                self.get_logger().warn('❌ Execution failed.')
        except Exception as e:
            self.get_logger().error(f'Exec service call failed: {e}')

def main():
    rclpy.init()
    node = xArmClient()
    # executor = MultiThreadedExecutor()
    # executor.add_node(node)
    # node.send_goal([0.377, 0.003, 0.488, 0.711, 0.006, 0.703, 0.001])



    # node.send_goal([0.0, -0.44, 0.1, 3.14, -1.57, -1.57]) # right
    # node.send_goal([0.0, -0.44, 0.1, 3.14, -1.57, -1.57]) # right
    # node.send_goal([0.0, -0.44, 0.1, 1.581, -1.514, -0.011]) # right
    # node.send_goal([0.0, -0.44, 0.1, 1.581, -1.514, -0.011]) # right
    # 1.880, -1.526, -0.309
    # node.send_goal([0.0, -0.44, 0.4, 3.14, -1.57, -1.57]) # right
    # node.send_goal([0.0, -0.44, 0.4, 3.14, -1.57, -1.57]) # right
    # node.send_goal([0.0, -0.44, 0.7, 3.14, -1.57, -1.57]) # right
    # node.send_goal([0.0, -0.44, 0.7, 3.14, -1.57, -1.57]) # right

    # node.send_goal([0.0, 0.44, 0.7, 0.0, -1.57, -1.57]) # left
    # node.send_goal([0.0, 0.44, 0.7, 0.0, -1.57, -1.57]) # left
    # node.send_goal([0.0, 0.44, 0.1, 3.14, -1.514, -0.011]) 
    # node.send_goal([0.0, 0.44, 0.1, 3.14, -1.514, -0.011]) 
    # node.send_goal([0.0, 0.44, 0.4, 0.0, -1.57, -1.57]) # left
    # node.send_goal([0.0, 0.44, 0.4, 0.0, -1.57, -1.57]) # left
    # node.send_goal([0.0, 0.44, 0.1, 0.0, -1.57, -1.57]) # left
    # node.send_goal([0.0, 0.44, 0.1, 0.0, -1.57, -1.57]) # left
    
    
    
    # node.send_goal([0.28, 0.0, 0.4, 2.580, -1.57, 0.562]) # origin
    # node.send_goal([0.28, 0.0, 0.4, 2.580, -1.57, 0.562]) # origin

    # node.send_goal([0.0, -0.44, 0.1, 3.14, -1.57, -1.57])  # right low
    # node.send_goal([0.0, -0.44, 0.1, 3.14, -1.57, -1.57])  # right low
    node.send_goal([0.0, -0.44, 0.4, 3.14, -1.57, -1.57])  # right mid
    node.send_goal([0.0, -0.44, 0.4, 3.14, -1.57, -1.57])  # right mid
    # node.send_goal([0.0, -0.44, 0.7, 3.14, -1.57, -1.57])  # right high
    # node.send_goal([0.0, -0.44, 0.7, 3.14, -1.57, -1.57])  # right high
    # node.send_goal([0.0,  0.44, 0.7, 0.0,  -1.57, -1.57])  # left high
    # node.send_goal([0.0,  0.44, 0.7, 0.0,  -1.57, -1.57])  # left high
    # node.send_goal([0.0,  0.44, 0.4, 0.0,  -1.57, -1.57])  # left mid
    # node.send_goal([0.0,  0.44, 0.4, 0.0,  -1.57, -1.57])  # left mid
    # node.send_goal([0.0,  0.44, 0.1, 0.0,  -1.57, -1.57])  # left low
    # node.send_goal([0.0,  0.44, 0.1, 0.0,  -1.57, -1.57])  # left low
    # node.send_goal([0.28, 0.0,  0.4, 2.580, -1.57, 0.562]) # origin
    # node.send_goal([0.28, 0.0,  0.4, 2.580, -1.57, 0.562]) # origin

    rclpy.shutdown()

if __name__ == '__main__':
    main()