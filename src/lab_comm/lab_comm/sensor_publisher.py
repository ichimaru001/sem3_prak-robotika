import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('amplitude', 1.0)
        self.declare_parameter('reliability', 'reliable')
        
        rate = self.get_parameter('rate_hz').value
        rel = (ReliabilityPolicy.RELIABLE 
               if self.get_parameter('reliability').value == 'reliable' 
               else ReliabilityPolicy.BEST_EFFORT)
        
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Float32, 'sensor_data', qos)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.k = 0
        self.get_logger().info(f'publishing at {rate} Hz, qos={rel.name}')

    def on_timer(self):
        amp = self.get_parameter('amplitude').value
        msg = Float32()
        msg.data = float(amp * math.sin(2 * math.pi * self.k / 50.0))
        self.pub.publish(msg)
        self.k += 1

def main():
    rclpy.init()
    node = SensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
