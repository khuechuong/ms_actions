import math
import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Point
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.duration import Duration
from rclpy.time import Time
from visualization_msgs.msg import Marker  # ← new

# ROS 2 package share lookup
from ament_index_python.packages import get_package_share_directory

from auto_actions.srv import GoToGoal

class GoToGoalServer(Node):
    def __init__(self):
        super().__init__('go_to_goal_server')
        pkg_share = get_package_share_directory('ms_iris')
        default_config = os.path.join(pkg_share, 'config', 'robert_config.yaml')
        
        # Declare ROS 2 parameter with default path
        self.declare_parameter('config_path', default_config)
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        cfg = yaml.safe_load(open(config_path))

        self.srv = self.create_service(GoToGoal, 'go_to_goal', self.go_to_goal_cb)
        self.cmd_pub = self.create_publisher(Twist, cfg['cmd_vel_topic'], 10)
        self.marker_pub = self.create_publisher(Marker, '/goal_marker', 10)  # ← new
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lin_tol = cfg['lin_tol']
        self.ang_tol = cfg['ang_tol']
        self.lin = cfg['lin_vel']
        self.ang = cfg['ang_vel']

        self.global_frame = cfg['global_frame']
        self.robot_frame = cfg['robot_center_frame']

        [_, _, yaw] = cfg['p0_rpy']
        self.arm_yaw = yaw + math.radians(90)
        # self.arm_yaw = 0.0

    def publish_goal_marker(self, x, y, reached=False):
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = 0
        # marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.type = Marker.ARROW

        marker.points.append(Point(x=x,y=y,z=0.0))
        marker.points.append(Point(x=x,y=y,z=0.5))

        marker.scale.x = 0.1  # shaft diameter
        marker.scale.y = 0.15   # head diameter
        marker.scale.z = 0.1   # arrow height
        
        # Green when reached, Red when active
        marker.color.r = 0.0 if not reached else 0.0
        marker.color.g = 1.0 if reached else 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        if not reached:
            marker.color.r = 1.0
        self.marker_pub.publish(marker)

    def get_pose(self, target_frame):
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.global_frame, target_frame, Time(), timeout=Duration(seconds=0.5))
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
            self.get_logger().info(f'Pose ({x:.2f}, {y:.2f})')
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

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def go_to_goal_cb(self, request, response):
        self.get_logger().info(f'Received goal ({request.x:.2f}, {request.y:.2f})')
        rate = self.create_rate(10)

        # States:
        # 0: Turn to face goal, then drive to goal
        # 1: Turn to final orientation
        # 3: Completed
        state = 0
        while rclpy.ok():
            if state == 3:
                break

            # Publish marker every loop — red while active
            self.publish_goal_marker(request.x, request.y, reached=(state == 3))
            # continue

            if state == 0:
                pose = self.get_pose(self.robot_frame)
                if pose is None:
                    rate.sleep()
                    continue

                x, y, theta = pose
                dx = request.x - x
                dy = request.y - y
                distance = math.hypot(dx, dy)
                target_angle = math.atan2(dy, dx)

                angle_error = self.normalize_angle(target_angle - theta)

                if distance < self.lin_tol:
                    self.stop_robot()
                    self.get_logger().info('Stage 0 Complete: Position reached. Moving to Stage 1 (Turn Setup).')
                    state = 1
                    rate.sleep()
                    continue

                if abs(angle_error) < self.ang_tol:
                    # print("lin err: ", distance)
                    lin_vel = self.lin
                    ang_vel = -0.1 * angle_error
                else:
                    # print("ang err: ", angle_error)
                    lin_vel = 0.0
                    ang_vel = self.ang if angle_error > 0 else -self.ang

                self.publish_velocity(lin_vel, ang_vel)

            elif state == 1:
                pose = self.get_pose(self.robot_frame)
                if pose is None:
                    print("POSE IS NONE!!!!")
                    rate.sleep()
                    continue

                _, _, theta = pose

                closest_angle = request.insp_yaw#math.pi if request.y > 0 else 0.0
                yaw_error = self.normalize_angle(closest_angle - theta)

                # print("yaw_err: ", yaw_error)

                if abs(yaw_error) < self.ang_tol:
                    self.stop_robot()
                    self.publish_goal_marker(request.x, request.y, reached=True)  # turns green
                    self.get_logger().info('Stage 1 Complete: Oriented for arm.')
                    state = 3
                    break

                ang_vel = self.ang if yaw_error > 0 else -self.ang
                self.publish_velocity(0.0, ang_vel)
            # print("gtg0")
            
            rate.sleep()
            # print("gtg1")
        # print("gtg2")
        if state == 3:
            response.success = True
        else:
            response.success = False
            self.stop_robot()
        # print("gtg3")

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
# import math
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, TransformStamped
# from tf2_ros import Buffer, TransformListener, TransformException
# from rclpy.duration import Duration
# from rclpy.time import Time

# from auto_actions.srv import GoToGoal  # Change to your actual package name

# class GoToGoalServer(Node):
#     def __init__(self):
#         super().__init__('go_to_goal_server')
#         self.srv = self.create_service(GoToGoal, 'go_to_goal', self.go_to_goal_cb)
#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         self.lin_tol = 0.05
#         self.ang_tol = 0.1
#         self.lin = 0.1
#         self.ang = 0.2

#         # TF Frames matching your exact hierarchy
#         self.global_frame = 'map' # 'odom'
#         self.robot_frame = 'robot_center' # 'base_link'
#         self.arm_frame = 'world' # 'world' acts as your arm base frame here

#     def get_pose(self, target_frame):
#         try:
#             tf: TransformStamped = self.tf_buffer.lookup_transform(
#                 #'odom', 'base_link', Time(), timeout=Duration(seconds=0.5))
#                 self.global_frame, target_frame, Time(), timeout=Duration(seconds=0.5))
#             x = tf.transform.translation.x
#             y = tf.transform.translation.y
#             q = tf.transform.rotation
#             yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
#             return x, y, yaw
#         except TransformException:
#             return None

#     def publish_velocity(self, lin, ang):
#         twist = Twist()
#         twist.linear.x = lin
#         twist.angular.z = ang
#         self.cmd_pub.publish(twist)

#     def stop_robot(self):
#         self.publish_velocity(0.0, 0.0)

#     def go_to_goal_cb(self, request, response):
#         self.get_logger().info(f'Received goal ({request.x:.2f}, {request.y:.2f})')
#         rate = self.create_rate(10)

#         # Sequential state machine states:
#         # 0: Go to Goal coordinates
#         # 1: Turn setup for arm
#         # 2: Move backwards for arm
#         # 3: Completed
#         state = 0
#         while rclpy.ok():
#             # State 0: Drive forward to goal coordinates
#             if state == 0:
#                 pose = self.get_pose(self.robot_frame)
#                 # print(pose)
#                 if pose is None:
#                     continue

#                 x, y, theta = pose
#                 dx = request.x - x
#                 dy = request.y - y
#                 distance = math.hypot(dx, dy)
#                 target_angle = math.atan2(dy, dx)
                
#                 # angle and normalize to [-π, π] 
#                 angle_error = math.atan2(math.sin(theta - target_angle), math.cos(theta - target_angle))
#                 # print(angle_error)
#                 if distance < self.lin_tol:
#                     self.stop_robot()
#                     self.get_logger().info('Stage 0 Complete: Position reached. Moving to Stage 1 (Turn Setup).')
#                     state = 1
#                     continue
                
#                 lin_vel = 0
#                 ang_vel = 0 
                
#                 if abs(angle_error) < self.ang_tol:
#                     print("lin er: ", distance)
#                     lin_vel = self.lin
#                     ang_vel = 0.0
#                 else:
#                     print("ang er: ", abs(angle_error))
#                     lin_vel = 0.0
#                     ang_vel = -self.ang if angle_error > 0 else self.ang
#                 self.publish_velocity(lin_vel, ang_vel)
#             elif state == 1:
#                 # State 1: Spin perpendicular to setup target orientation
#                 pose = self.get_pose(self.robot_frame)
#                 if pose is None:
#                     print("POSE IS NONE!!!!")
#                     rate.sleep()
#                     continue

#                 _, _, theta = pose

#                 # Perpendicular strategy based on request orientation from your ROS 1 script
#                 closest_angle = math.pi if request.y > 0 else 0.0
#                 yaw_error = self.normalize_angle(theta - closest_angle)

#                 if abs(yaw_error) < self.ang_tol:
#                     self.stop_robot()
#                     self.get_logger().info('Stage 1 Complete: Oriented for arm. Moving to Stage 2 (Backing up).')
#                     """ TODO: CHANGE STATE"""
#                     state = 3
#                     break
#                     # continue
#             # State 2: Back up until the 'world' frame reaches target point threshold
#             elif state == 2:
#                 arm_pose = self.get_pose(self.arm_frame)
#                 if arm_pose is None:
#                     rate.sleep()
#                     print("POSE IS NONE!")
#                     continue

#                 arm_x, arm_y, _ = arm_pose

#                 # Distance checking calculation based on your legacy script logic (X-axis comparison)
#                 distance = abs(arm_x - request.x)
#                 self.get_logger().info(f'Arm Base (world) Distance to Goal X: {distance:.3f}', throttle_duration_sec=1.0)

#                 if distance < self.arm_lin_tol:
#                     self.stop_robot()
#                     self.get_logger().info('Stage 2 Complete: Arm base is positioned correctly at the target.')
#                     state = 3
#                     break

#                 # Moving backward at your specified baseline linear velocity
#                 self.publish_velocity(-self.lin, 0.0)
#             rate.sleep()

#         if state == 3:
#             response.success = True
#         else:
#             response.success = False
#             self.stop_robot()

#         return response
    
# from rclpy.executors import MultiThreadedExecutor
# def main():
#     rclpy.init()
#     node = GoToGoalServer()
#     executor = MultiThreadedExecutor()
#     executor.add_node(node)
#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()



# if __name__ == '__main__':
#     main()