import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Int32, Bool
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import quaternion_from_euler

from xarm_msgs.srv import PlanPose, PlanExec, SetInt16
from auto_actions.srv import Inspect, Er

class InspectServer(Node):
    """
    NOTE: 
    server inputs (req.robot_pose and req.inspect_area) should 
    already expressed in the xArm frame (default: 'world')
    """
    def __init__(self):
        super().__init__('inspect_server')
        # sub node for xArm services
        self.sub_node = rclpy.create_node('sub_node')
        self.srv = self.create_service(Inspect, 'inspect', self.inspect_cb)
        # xArm services
        self.plan_cli = self.sub_node.create_client(PlanPose, '/xarm_pose_plan')
        self.exec_cli = self.sub_node.create_client(PlanExec, '/xarm_exec_plan')
        self.state_cli = self.sub_node.create_client(SetInt16, '/xarm/set_state')
        # ER service
        # self.er_cli = self.sub_node.create_client(Er, 'er_service')
        # contact-based subscribers
        self.ultrasonic_sub = self.sub_node.create_subscription(Int32, '/CAIS/distance_mm', self.ultrasonic_cb, 10)
        self.limit_sub = self.sub_node.create_subscription(Int32, '/CAIS/limit', self.limit_cb, 10)
        # rgb-t subscribers
        self.rgb_sub = self.sub_node.create_subscription(Int32, '/oak/rgb/image_rect_color', self.rgb_cb, 10)
        self.t_sub = self.sub_node.create_subscription(Int32, '/oak/thermal/image_rect', self.t_cb, 10)
        # publisher
        self.pump_pub = self.create_publisher(Bool, '/CAIS/pump', 10)

        self.ultrasonic = 10
        self.false_count = 0
        self.false_threshold = 3  # Number of consecutive False readings required
        self.getER = False
        self.finishER = False

        # Wait until both services are ready
        self.get_logger().info('Waiting for xarm services...')
        if not self.plan_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('PlanPose service not available')
            rclpy.shutdown(); return
        if not self.exec_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('PlanExec service not available')
            rclpy.shutdown(); return
        if not self.state_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('setState service not available')
            rclpy.shutdown(); return
        # if not self.er_cli.wait_for_service(timeout_sec=5.0):
        #     self.get_logger().error('ER service not available')
        #     rclpy.shutdown(); return
        self.get_logger().info('Services available, starting sequence')
        self.setState(0)

    def inspect_cb(self, req, res):
        x = 0.0
        y = 0.44
        z = min([0.1, 0.4, 0.7], key=lambda t: abs(req.inspect_area.z - t))
        # determine if point is left or right
        rx, ry, yaw = req.robot_pose.x, req.robot_pose.y, req.robot_pose.z
        gx, gy = req.inspect_area.x, req.inspect_area.y
        c = math.cos(yaw)*(gy-ry) - math.sin(yaw)*(gx-rx)
        left = 1 if c >= 0 else -1
        if left == 1:
            qx, qy, qz, qw = quaternion_from_euler(0.0, -1.57, -1.57)
            self.get_logger().info('LEFT')
        else:
            qx, qy, qz, qw = quaternion_from_euler(3.14, -1.57, -1.57)
            self.get_logger().info('RIGHT')
        ## Visual Inspection
        self.send_goal([x, (y * left), z, qx, qy, qz, qw])
        # if not self.send_goal([x, (y * left), z, qx, qy, qz, qw]):
        #     res.success = False
        #     return res
        # TODO: save pic       

        ## water pump  
        self.pump_pub.publish(Bool(data=True))  
        self.get_logger().info("Spraying water")
        time.sleep(1.4)  # blocks executor thread
        self.get_logger().info("Waiting")
        self.pump_pub.publish(Bool(data=False))
        time.sleep(15)

        ## take thermal data

        ## contact-base sensor
        # getting closer
        if not self.getER:
            while self.ultrasonic > 0.1 * 1000:
                y += 0.1
                self.send_goal([x, (y * left), z, qx, qy, qz, qw])
            # print("closer")
            # smaller movements when closer
            while not self.getER:
                y += 0.01
                self.send_goal([x, (y * left), z, qx, qy, qz, qw])
                # print(self.getER)
                # if self.getER:
                #     break
        # prevent another abort if limit switch still pressed
        self.finishER = True
        self.getER = False        
        
        # print("Take ER readings")
        # take ER readings     
        data = -1
        attempts = 0
        self.setState(0)
        while data == -1 and attempts < 3:
            data = self.getERReading()
            attempts += 1
        #     print(data)
        #     print(data)
        #     if data != -1:
        #         print("4")
        #         break
        # print("4")
        self.get_logger().info(f"ER data: {data:.2f}")
        try:
            # recover from abort
            self.getER = False
            # prevent another abort if limit switch still pressed
            self.finishER = True
            ## withdraw
            self.setState(0)
            # print(self.getER)
        except Exception as e:
            self.get_logger().error(f"debug block exception: {e}")
        # back twice
        for _ in range(2):
            y -= 0.1
            self.send_goal([x, (y * left), z, qx, qy, qz, qw])
        # go to first goal
        self.send_goal([x, (0.44 * left), z, qx, qy, qz, qw])
        self.finishER = True
        res.success = True
        res.er_reading = data
        return res

    def send_goal(self, goal):
        # print(goal)
        plan_req = PlanPose.Request()
        plan_req.target = Pose()
        plan_req.target.position.x, plan_req.target.position.y, plan_req.target.position.z = goal[:3]
        
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
        # print(plan_req)
        # future_plan = self.plan_cli.call_async(plan_req)
        # rclpy.spin_until_future_complete(self.sub_node, future_plan)
        # if not future_plan.result().success:
        #     self.get_logger().info('❌ Failed to plan goal.')
        #     print("failed")
        #     return False
        f_plan = self.plan_cli.call_async(plan_req)
        if not self._wait_future_with_abort(f_plan, self.sub_node, abort_pred=lambda: self.getER, max_sec=10.0):
            self.get_logger().warn("Planning aborted or timed out.")
            return False
        if not f_plan.result().success:
            self.get_logger().warn("Planning failed.")
            return False
        # Execute
        self.get_logger().info('✅ Plan successful, executing trajectory...')
        exec_req = PlanExec.Request()
        exec_req.wait = True  # wait until motion completes
        # future_exec = self.exec_cli.call_async(exec_req)
        # rclpy.spin_until_future_complete(self.sub_node, future_exec)
        # try:
        #     if future_exec.result().success:
        #         self.get_logger().info('✅ Execution completed successfully.')
        #         return True
        #     else:
        #         self.get_logger().warn('❌ Execution failed.')
        #         return False
        # except Exception as e:
        #     self.get_logger().error(f'Exec service call failed: {e}')  
        #     return False  
        f_exec = self.exec_cli.call_async(exec_req)
        ok = self._wait_future_with_abort(f_exec, self.sub_node, abort_pred=lambda: self.getER, max_sec=60.0)
        if not ok:
            self.get_logger().warn("Execution aborted (limit switch / ER).")
            return False
        if not f_exec.result().success:
            self.get_logger().warn("Execution failed.")
            return False
        self.get_logger().info("Execution completed.")
        return True
        
    def _wait_future_with_abort(self, future, node_to_spin, abort_pred, step=0.05, max_sec=None):
        start = time.time()
        while rclpy.ok():
            # abort first so we bail immediately when limit hits
            if abort_pred():
                self.get_logger().warning("wait(): abort predicate TRUE (limit/ER).")
                return False
            if future.done():
                try:
                    _ = future.result()  # surface service exceptions
                except Exception as e:
                    self.get_logger().error(f"wait(): future raised: {e!r}")
                    return False
                return True
            rclpy.spin_once(node_to_spin, timeout_sec=step)
            if max_sec is not None and (time.time() - start) > max_sec:
                self.get_logger().warning(f"wait(): TIMEOUT after {max_sec:.2f}s.")
                return False
        return False
        
    def setState(self, state: int):
        req = SetInt16.Request()
        req.data = state
        future_exec = self.state_cli.call_async(req)
        time.sleep(0.05)
        self.get_logger().info(f"Set State to {state}")
        # rclpy.spin_until_future_complete(self.sub_node, future_exec)
        # try:
        #     ret = future_exec.result().ret
        #     msg = future_exec.result().message
        #     self.get_logger().info(f"Set State to {state}. Ret: {ret}. Msg: {msg}")
        # except Exception as e:
        #     self.get_logger().error(f'Set State service call failed: {e}')  
    
    def getERReading(self):
        req = Er.Request()
        future = self.er_cli.call_async(req)
        rclpy.spin_until_future_complete(self.sub_node, future)

        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call exception: {e}")
            return None     
        
        # Expect your server to set resp.er_reading (float)
        if hasattr(resp, 'er_reading'):
            self.get_logger().info(f"er_reading avg = {resp.er_reading:.3f}")
            # print("2")
            return float(resp.er_reading)
        else:
            self.get_logger().error("Response missing 'er_reading' field.")
            return -1.0

    def ultrasonic_cb(self, msg):
        self.ultrasonic = msg.data

    def limit_cb(self, msg):
        if self.finishER:
            return
        limit_switch = -1
        if not self.getER:
            limit_switch = msg.data
            if limit_switch == 3:
                self.false_count += 1
            else:
                self.false_count = 0
        if self.false_count >= self.false_threshold and self.getER == False:   
            self.getER = True         
            self.setState(4)
            self.get_logger().info("Set Arm State to 4 to stop")            
            # reset
            self.false_count = 0
    
    def rgb_cb(self, data):
        pass

    def t_cb(self, data):
        pass

from rclpy.executors import MultiThreadedExecutor
def main():
    rclpy.init()    
    executor = MultiThreadedExecutor(num_threads=2)
    node = InspectServer()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()