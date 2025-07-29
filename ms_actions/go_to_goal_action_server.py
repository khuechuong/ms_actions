import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.duration import Duration
from rclpy.time import Time

from auto_actions.srv import GoToGoal  # Change to your actual package name

class GoToGoalServer(Node):
    def __init__(self):
        super().__init__('go_to_goal_server')
        self.srv = self.create_service(GoToGoal, 'go_to_goal', self.go_to_goal_cb)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lin_tol = 0.1
        self.ang_tol = 0.1
        self.lin = 0.2
        self.ang = 1.0

    def get_pose(self):
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                'odom', 'base_link', Time(), timeout=Duration(seconds=0.5))
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
            return x, y, yaw
        except TransformException:
            return None

    def publish_velocity(self, lin, ang):
        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)

    def go_to_goal_cb(self, request, response):
        self.get_logger().info(f'Received goal ({request.x:.2f}, {request.y:.2f})')
        rate = self.create_rate(10)

        while rclpy.ok():
            pose = self.get_pose()
            # print(pose)
            if pose is None:
                continue

            x, y, theta = pose
            dx = request.x - x
            dy = request.y - y
            distance = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)
            
            # angle and normalize to [-π, π] 
            angle_error = math.atan2(math.sin(theta - target_angle), math.cos(theta - target_angle))
            print(angle_error)

            if distance < self.lin_tol:
                self.stop_robot()
                self.get_logger().info('Goal reached.')
                response.success = True
                return response
            
            lin_vel = 0
            ang_vel = 0 
            if abs(angle_error) < self.ang_tol:
                lin_vel = self.lin
                ang_vel = 0.0
            else:
                lin_vel = 0.0
                ang_vel = -self.ang if angle_error > 0 else self.ang
            self.publish_velocity(lin_vel, ang_vel)

            rate.sleep()

        response.success = False
        return response
    
from rclpy.executors import MultiThreadedExecutor
def main():
    rclpy.init()
    node = GoToGoalServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()