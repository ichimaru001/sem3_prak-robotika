import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ResetService(Node):
    def __init__(self):
        super().__init__('reset_service')
        self.counter = 0
        self.srv = self.create_service(
            Trigger, 'reset_system', self.on_reset)

    def on_reset(self, request, response):
        old = self.counter
        self.counter = 0
        response.success = True
        response.message = f'counter reset from {old}'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ResetService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
