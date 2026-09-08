import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class MotionActionServer(Node):
    def __init__(self):
        super().__init__('motion_action_server')
        self.srv = ActionServer(
            self, Fibonacci, 'execute_motion',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel)

    def on_goal(self, goal_request):
        if goal_request.order > 25:
            self.get_logger().warn('goal rejected: order too large')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        feedback = Fibonacci.Feedback()
        feedback.sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()
            feedback.sequence.append(
                feedback.sequence[i] + feedback.sequence[i-1])
            goal_handle.publish_feedback(feedback)
            time.sleep(0.5)
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback.sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MotionActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
