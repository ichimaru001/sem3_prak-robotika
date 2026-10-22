import math
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))
def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
class ClosedLoop(Node):
    def __init__(self):
        super().__init__('closed_loop')
        self.declare_parameter('trajectory', 'line')
        self.declare_parameter('k_rho', 0.6)
        self.declare_parameter('k_alpha', 1.5)
        self.declare_parameter('v_max', 0.15)
        self.declare_parameter('w_max', 1.0)
        self.declare_parameter('tol_pos', 0.03)
        self.declare_parameter('tol_heading', math.radians(5.0))
        self.pose = None
        self.start_pose = None
        self.last_odom_time = None
        self.waypoints = []
        self.goal_index = 0
        self.finished = False
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_timer(0.05, self.control)
    def odom_callback(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        self.last_odom_time = time.monotonic()
        if self.start_pose is None:
            self.start_pose = self.pose
            self.generate_waypoints()
            self.get_logger().info(f'Start pose = {self.start_pose}')
    def local_to_world(self, lx, ly):
        x0, y0, th0 = self.start_pose
        wx = x0 + math.cos(th0) * lx - math.sin(th0) * ly
        wy = y0 + math.sin(th0) * lx + math.cos(th0) * ly
        return (wx, wy)
    def generate_waypoints(self):
        trajectory = self.get_parameter('trajectory').get_parameter_value().string_value
        if trajectory == 'line':
            self.waypoints = [self.local_to_world(1.0, 0.0)]
        elif trajectory == 'square':
            local_points = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
            self.waypoints = [self.local_to_world(x, y) for x, y in local_points]
        elif trajectory == 'figure8':
            local_points = []
            R = 0.5
            N = 32
            for i in range(1, N + 1):
                angle = 2.0 * math.pi * i / N
                x = R * math.sin(angle)
                y = R * (1.0 - math.cos(angle))
                local_points.append((x, y))
            for i in range(1, N + 1):
                angle = 2.0 * math.pi * i / N
                x = R * math.sin(angle)
                y = -R * (1.0 - math.cos(angle))
                local_points.append((x, y))
            self.waypoints = [self.local_to_world(x, y) for x, y in local_points]
        elif trajectory == 'rotate':
            self.waypoints = []
        else:
            self.get_logger().error('Unknown trajectory')
    def stop(self):
        self.pub.publish(Twist())
    def finish(self):
        if not self.finished:
            self.finished = True
            self.stop()
            self.get_logger().info('FINISHED')
    def control(self):
        if self.finished:
            self.stop()
            return
        if self.last_odom_time is None:
            self.stop()
            return
        if time.monotonic() - self.last_odom_time > 0.5:
            self.get_logger().error('ODOM timeout > 0.5 s. STOP!')
            self.stop()
            return
        if self.pose is None or self.start_pose is None:
            return
        x, y, th = self.pose
        _, _, start_th = self.start_pose
        trajectory = self.get_parameter('trajectory').get_parameter_value().string_value
        k_rho = float(self.get_parameter('k_rho').value)
        k_alpha = float(self.get_parameter('k_alpha').value)
        v_max = float(self.get_parameter('v_max').value)
        w_max = float(self.get_parameter('w_max').value)
        tol_pos = float(self.get_parameter('tol_pos').value)
        tol_heading = float(self.get_parameter('tol_heading').value)
        cmd = Twist()
        if trajectory == 'rotate':
            target_heading = wrap(start_th + math.pi)
            error = wrap(target_heading - th)
            if abs(error) < tol_heading:
                self.finish()
                return
            w = k_alpha * error
            w = max(-w_max, min(w_max, w))
            cmd.angular.z = w
            self.pub.publish(cmd)
            return
        if self.goal_index < len(self.waypoints):
            gx, gy = self.waypoints[self.goal_index]
            rho = math.hypot(gx - x, gy - y)
            alpha = wrap(math.atan2(gy - y, gx - x) - th)
            if rho < tol_pos:
                self.goal_index += 1
                self.stop()
                return
            w = k_alpha * alpha
            w = max(-w_max, min(w_max, w))
            if abs(alpha) > 0.4:
                v = 0.0
            else:
                v = min(k_rho * rho, v_max)
            cmd.linear.x = v
            cmd.angular.z = w
            self.pub.publish(cmd)
            return
        heading_error = wrap(start_th - th)
        if abs(heading_error) < tol_heading:
            self.finish()
            return
        w = k_alpha * heading_error
        w = max(-w_max, min(w_max, w))
        cmd.angular.z = w
        self.pub.publish(cmd)
def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ClosedLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            node.stop()
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()
