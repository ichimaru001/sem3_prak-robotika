import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ProcessingNode(Node):
    def __init__(self):
        super().__init__('processing_node')
        self.declare_parameter('threshold', 0.5)
        self.count_above = 0
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(
            Float32, 'sensor_data', self.on_data, qos)
        self.pub = self.create_publisher(Float32, 'processed_data', 10)
        self.add_on_set_parameters_callback(self.on_param)

    def on_param(self, params):
        for p in params:
            if p.name == 'threshold' and p.value < 0.0:
                return SetParametersResult(
                    successful=False, reason='threshold must be >= 0')
        return SetParametersResult(successful=True)

    def on_data(self, msg):
        th = self.get_parameter('threshold').value
        if abs(msg.data) > th:
            self.count_above += 1
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ProcessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
