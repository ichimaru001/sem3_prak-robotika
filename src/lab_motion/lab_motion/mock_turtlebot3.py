import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

class MockTurtleBot3(Node):
    """
    Kinematic simulator node for TurtleBot3 Burger.
    Enables fully functional offline experimentation without connecting to the robot's physical WiFi.
    """
    def __init__(self):
        super().__init__('mock_turtlebot3')

        # TurtleBot3 Burger Parameters
        self.declare_parameter('wheel_radius', 0.033)   # r in meters
        self.declare_parameter('wheel_base', 0.160)     # b in meters
        self.declare_parameter('max_lin_vel', 0.22)     # v_max in m/s
        self.declare_parameter('max_ang_vel', 2.84)     # w_max in rad/s
        self.declare_parameter('max_wheel_vel', 6.387)  # Dynamixel XL430-W250 max (~61 rpm)
        self.declare_parameter('publish_rate', 30.0)    # 30 Hz standard rate
        self.declare_parameter('simulate_noise', False)
        self.declare_parameter('simulate_slip', False)

        self.r = float(self.get_parameter('wheel_radius').value)
        self.b = float(self.get_parameter('wheel_base').value)
        self.max_lin_vel = float(self.get_parameter('max_lin_vel').value)
        self.max_ang_vel = float(self.get_parameter('max_ang_vel').value)
        self.max_wheel_vel = float(self.get_parameter('max_wheel_vel').value)
        rate = float(self.get_parameter('publish_rate').value)

        # Pose & State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.v_act = 0.0
        self.w_act = 0.0

        # Wheel angles & velocities
        self.wheel_l_pos = 0.0
        self.wheel_r_pos = 0.0
        self.wheel_l_vel = 0.0
        self.wheel_r_vel = 0.0

        self.last_cmd_time = None
        self.last_update_time = time.monotonic()

        # Publishers & Subscribers
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.sub_reset = self.create_subscription(Empty, '/reset_odom', self.reset_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_joint = self.create_publisher(JointState, '/joint_states', 10)

        dt = 1.0 / rate
        self.timer = self.create_timer(dt, self.update_kinematics)
        self.get_logger().info(f'MockTurtleBot3 running: r={self.r} m, b={self.b} m at {rate} Hz')

    def cmd_callback(self, msg: Twist):
        self.v_cmd = msg.linear.x
        self.w_cmd = msg.angular.z
        self.last_cmd_time = time.monotonic()

    def reset_callback(self, msg: Empty):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.v_act = 0.0
        self.w_act = 0.0
        self.wheel_l_pos = 0.0
        self.wheel_r_pos = 0.0
        self.wheel_l_vel = 0.0
        self.wheel_r_vel = 0.0
        self.get_logger().info('MockTurtleBot3 pose reset to (0,0,0)')

    def update_kinematics(self):
        now = time.monotonic()
        dt = now - self.last_update_time
        self.last_update_time = now

        # Watchdog: Stop if no cmd_vel received in 0.5 s
        if self.last_cmd_time is None or (now - self.last_cmd_time > 0.5):
            self.v_cmd = 0.0
            self.w_cmd = 0.0

        # Inverse Kinematics: calculate target wheel angular velocities
        # omega_R = (v + b*w/2) / r
        # omega_L = (v - b*w/2) / r
        w_r_target = (self.v_cmd + (self.b * self.w_cmd / 2.0)) / self.r
        w_l_target = (self.v_cmd - (self.b * self.w_cmd / 2.0)) / self.r

        # Motor saturation check (Dynamixel XL430-W250 limit)
        w_r = max(-self.max_wheel_vel, min(self.max_wheel_vel, w_r_target))
        w_l = max(-self.max_wheel_vel, min(self.max_wheel_vel, w_l_target))

        self.wheel_r_vel = w_r
        self.wheel_l_vel = w_l

        # Forward Kinematics: actual robot velocity from wheel speeds
        self.v_act = self.r * (w_r + w_l) / 2.0
        self.w_act = self.r * (w_r - w_l) / self.b

        # Integrate wheel positions
        self.wheel_r_pos += w_r * dt
        self.wheel_l_pos += w_l * dt

        # Integrate 2D Unicycle pose
        delta_th = self.w_act * dt
        delta_s = self.v_act * dt

        # Midpoint orientation integration for high accuracy
        mid_th = self.th + delta_th / 2.0
        self.x += delta_s * math.cos(mid_th)
        self.y += delta_s * math.sin(mid_th)
        self.th = math.atan2(math.sin(self.th + delta_th), math.cos(self.th + delta_th))

        stamp = self.get_clock().now().to_msg()

        # Publish /joint_states
        joint_msg = JointState()
        joint_msg.header.stamp = stamp
        joint_msg.name = ['wheel_left_joint', 'wheel_right_joint']
        joint_msg.position = [float(self.wheel_l_pos), float(self.wheel_r_pos)]
        joint_msg.velocity = [float(self.wheel_l_vel), float(self.wheel_r_vel)]
        joint_msg.effort = [0.0, 0.0]
        self.pub_joint.publish(joint_msg)

        # Publish /odom
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = float(self.x)
        odom_msg.pose.pose.position.y = float(self.y)
        odom_msg.pose.pose.position.z = 0.0

        # Orientation quaternion: yaw around z-axis
        half_th = self.th / 2.0
        odom_msg.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(half_th),
            w=math.cos(half_th)
        )
        odom_msg.twist.twist.linear.x = float(self.v_act)
        odom_msg.twist.twist.angular.z = float(self.w_act)

        self.pub_odom.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockTurtleBot3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
