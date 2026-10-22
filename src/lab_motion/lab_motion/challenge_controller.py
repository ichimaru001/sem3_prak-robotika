import os
import math
import time
import yaml
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

class ChallengeController(Node):
    """
    Section 6.9 Engineering Challenge Controller.
    Executes: Start -> 1 m forward -> 90 deg left -> 1 m forward -> Stop (publishes 'FINISHED' to /motion_status).
    Features:
    - Reads waypoints from external YAML file
    - Orientation gating to prevent wide arcs
    - Slew-rate limiter for smooth acceleration/deceleration (bonus feature)
    - Watchdog (> 0.5 s odom timeout safe stop)
    - Safe stop on shutdown (publishes 0 Twist on Ctrl+C)
    - Reports quantitative errors to log and /motion_status
    """
    def __init__(self):
        super().__init__('challenge_controller')

        # Parameter declarations
        default_yaml = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), 'config', 'waypoints.yaml'
        )
        self.declare_parameter('waypoints_file', default_yaml)
        self.declare_parameter('use_slew_rate', True)
        self.declare_parameter('v_max', 0.15)
        self.declare_parameter('w_max', 1.0)
        self.declare_parameter('k_rho', 0.6)
        self.declare_parameter('k_alpha', 1.5)
        self.declare_parameter('tol_pos', 0.03)
        self.declare_parameter('tol_heading', math.radians(4.0))

        # Publishers & Subscribers
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/motion_status', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # State variables
        self.pose = None
        self.start_pose = None
        self.last_odom_time = None
        self.finished = False

        # Segments & execution index
        self.segments = []
        self.segment_index = 0
        self.final_target_world = None
        self.final_target_heading = None

        # Slew rate state
        self.current_v = 0.0
        self.current_w = 0.0
        self.last_control_time = time.monotonic()
        self.max_accel_lin = 0.20   # m/s^2
        self.max_accel_ang = 1.0    # rad/s^2

        # Load YAML waypoints
        self.load_yaml_config()

        # 20 Hz control loop
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('ChallengeController initialized.')

    def load_yaml_config(self):
        yaml_path = self.get_parameter('waypoints_file').get_parameter_value().string_value
        # Fallback paths
        paths_to_check = [
            yaml_path,
            '/ws/config/waypoints.yaml',
            '/ws/src/lab_motion/config/waypoints.yaml',
            os.path.expanduser('~/robotics-turtlebot3/config/waypoints.yaml')
        ]
        chosen_path = None
        for p in paths_to_check:
            if p and os.path.isfile(p):
                chosen_path = p
                break

        if not chosen_path:
            self.get_logger().warn(f'YAML config not found at {yaml_path}, using default challenge segments.')
            self.raw_segments = [
                {'type': 'waypoint', 'x': 1.0, 'y': 0.0},
                {'type': 'heading', 'yaw_deg': 90.0},
                {'type': 'waypoint', 'x': 1.0, 'y': 1.0},
                {'type': 'heading', 'yaw_deg': 90.0}
            ]
            return

        self.get_logger().info(f'Loading waypoints from YAML: {chosen_path}')
        with open(chosen_path, 'r') as f:
            data = yaml.safe_load(f)

        cfg = data.get('challenge_waypoints', {})
        self.raw_segments = cfg.get('segments', [])
        if 'tolerance_pos' in cfg:
            self.set_parameters([rclpy.parameter.Parameter('tol_pos', rclpy.Parameter.Type.DOUBLE, float(cfg['tolerance_pos']))])
        if 'tolerance_heading_deg' in cfg:
            self.set_parameters([rclpy.parameter.Parameter('tol_heading', rclpy.Parameter.Type.DOUBLE, math.radians(float(cfg['tolerance_heading_deg'])))])
        if 'v_max' in cfg:
            self.set_parameters([rclpy.parameter.Parameter('v_max', rclpy.Parameter.Type.DOUBLE, float(cfg['v_max']))])
        if 'w_max' in cfg:
            self.set_parameters([rclpy.parameter.Parameter('w_max', rclpy.Parameter.Type.DOUBLE, float(cfg['w_max']))])
        if 'use_slew_rate' in cfg:
            self.set_parameters([rclpy.parameter.Parameter('use_slew_rate', rclpy.Parameter.Type.BOOL, bool(cfg['use_slew_rate']))])
        if 'max_accel_lin' in cfg:
            self.max_accel_lin = float(cfg['max_accel_lin'])
        if 'max_accel_ang' in cfg:
            self.max_accel_ang = float(cfg['max_accel_ang'])

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        self.last_odom_time = time.monotonic()

        if self.start_pose is None:
            self.start_pose = self.pose
            self.generate_world_segments()
            self.get_logger().info(f'Start pose locked = {self.start_pose}')

    def local_to_world(self, lx, ly):
        x0, y0, th0 = self.start_pose
        wx = x0 + math.cos(th0) * lx - math.sin(th0) * ly
        wy = y0 + math.sin(th0) * lx + math.cos(th0) * ly
        return (wx, wy)

    def generate_world_segments(self):
        x0, y0, th0 = self.start_pose
        self.segments = []
        for seg in self.raw_segments:
            seg_type = seg.get('type')
            if seg_type == 'waypoint':
                wx, wy = self.local_to_world(float(seg['x']), float(seg['y']))
                self.segments.append(('waypoint', wx, wy))
            elif seg_type == 'heading':
                target_th = wrap(th0 + math.radians(float(seg['yaw_deg'])))
                self.segments.append(('heading', target_th))

        # Expected final goal pose in world frame
        # Last waypoint was local (1.0, 1.0) and heading was +90 deg (pi/2)
        self.final_target_world = self.local_to_world(1.0, 1.0)
        self.final_target_heading = wrap(th0 + math.radians(90.0))
        self.get_logger().info(f'Generated {len(self.segments)} challenge segments. Target = {self.final_target_world}, heading = {self.final_target_heading:.3f}')

    def stop(self):
        self.pub_cmd.publish(Twist())
        self.current_v = 0.0
        self.current_w = 0.0

    def apply_slew_rate(self, target_v, target_w, dt):
        use_slew = self.get_parameter('use_slew_rate').get_parameter_value().bool_value
        if not use_slew or dt <= 0:
            self.current_v = target_v
            self.current_w = target_w
            return target_v, target_w

        # Linear acceleration limit
        max_dv = self.max_accel_lin * dt
        dv = target_v - self.current_v
        self.current_v += max(-max_dv, min(max_dv, dv))

        # Angular acceleration limit
        max_dw = self.max_accel_ang * dt
        dw = target_w - self.current_w
        self.current_w += max(-max_dw, min(max_dw, dw))

        return self.current_v, self.current_w

    def finish_challenge(self):
        if not self.finished:
            self.finished = True
            self.stop()
            x, y, th = self.pose
            gx, gy = self.final_target_world
            gth = self.final_target_heading

            e_p = math.hypot(gx - x, gy - y)
            e_th_deg = abs(wrap(gth - th)) * (180.0 / math.pi)

            status_msg = (
                f'FINISHED: Final Error e_p = {e_p:.4f} m (<= 0.05 m: {e_p <= 0.05}), '
                f'e_theta = {e_th_deg:.2f} deg (<= 5.0 deg: {e_th_deg <= 5.0})'
            )
            self.get_logger().info('=' * 60)
            self.get_logger().info(status_msg)
            self.get_logger().info('=' * 60)

            # Publish status to /motion_status
            msg = String()
            msg.data = status_msg
            self.pub_status.publish(msg)

    def control_loop(self):
        if self.finished:
            self.stop()
            return

        now = time.monotonic()
        dt = now - self.last_control_time
        self.last_control_time = now

        # Watchdog: Stop if odom lost > 0.5 s
        if self.last_odom_time is None:
            self.stop()
            return
        if now - self.last_odom_time > 0.5:
            self.get_logger().error('ODOM timeout > 0.5 s! Emergency stop!')
            self.stop()
            return

        if self.pose is None or self.start_pose is None:
            return

        x, y, th = self.pose
        k_rho = float(self.get_parameter('k_rho').value)
        k_alpha = float(self.get_parameter('k_alpha').value)
        v_max = float(self.get_parameter('v_max').value)
        w_max = float(self.get_parameter('w_max').value)
        tol_pos = float(self.get_parameter('tol_pos').value)
        tol_heading = float(self.get_parameter('tol_heading').value)

        # Check if all segments completed
        if self.segment_index >= len(self.segments):
            self.finish_challenge()
            return

        current_segment = self.segments[self.segment_index]
        seg_type = current_segment[0]

        target_v = 0.0
        target_w = 0.0

        if seg_type == 'waypoint':
            _, gx, gy = current_segment
            rho = math.hypot(gx - x, gy - y)
            alpha = wrap(math.atan2(gy - y, gx - x) - th)

            if rho < tol_pos:
                self.get_logger().info(f'Reached waypoint {self.segment_index}: ({gx:.2f}, {gy:.2f}), e_p={rho:.3f} m')
                self.segment_index += 1
                self.stop()
                return

            w = max(-w_max, min(w_max, k_alpha * alpha))
            # Orientation gate: if heading error > 0.4 rad (~23 deg), turn in place first
            if abs(alpha) > 0.4:
                v = 0.0
            else:
                v = min(k_rho * rho, v_max)

            target_v = v
            target_w = w

        elif seg_type == 'heading':
            _, target_th = current_segment
            error = wrap(target_th - th)

            if abs(error) < tol_heading:
                self.get_logger().info(f'Reached heading {self.segment_index}: {math.degrees(target_th):.1f} deg, error={math.degrees(error):.2f} deg')
                self.segment_index += 1
                self.stop()
                return

            w = max(-w_max, min(w_max, k_alpha * error))
            target_v = 0.0
            target_w = w

        # Apply slew-rate limiter
        cmd_v, cmd_w = self.apply_slew_rate(target_v, target_w, dt)

        cmd = Twist()
        cmd.linear.x = float(cmd_v)
        cmd.angular.z = float(cmd_w)
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ChallengeController()
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
