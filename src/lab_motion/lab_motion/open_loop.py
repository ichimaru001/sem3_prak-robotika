import math
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from geometry_msgs.msg import Twist
class OpenLoop(Node):
    def __init__(self):
        super().__init__('open_loop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.declare_parameter('trajectory', 'line')
        self.declare_parameter('v', 0.1)
        self.declare_parameter('w', 0.5)
    def send(self, v, w, duration):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        end = time.monotonic() + duration
        while time.monotonic() < end and rclpy.ok():
            self.pub.publish(msg)
            time.sleep(0.05)
        self.stop()
        time.sleep(0.3)
    def stop(self):
        self.pub.publish(Twist())
    def run(self):
        trajectory = self.get_parameter('trajectory').get_parameter_value().string_value
        v = float(self.get_parameter('v').value)
        w = float(self.get_parameter('w').value)
        self.get_logger().info(f'Running open-loop trajectory: {trajectory}')
        if trajectory == 'line':
            duration = 1.0 / v
            self.send(v, 0.0, duration)
        elif trajectory == 'rotate':
            duration = math.pi / w
            self.send(0.0, w, duration)
        elif trajectory == 'square':
            for _ in range(4):
                self.send(v, 0.0, 1.0 / v)
                self.send(0.0, w, math.pi / 2.0 / w)
        elif trajectory == 'figure8':
            w_fig = v / 0.5
            circle_time = 2.0 * math.pi / w_fig
            self.send(v, w_fig, circle_time)
            self.send(v, -w_fig, circle_time)
        else:
            self.get_logger().error('trajectory harus line, rotate, square, atau figure8')
        self.stop()
def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = OpenLoop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
