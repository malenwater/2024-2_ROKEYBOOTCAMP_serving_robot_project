from rclpy.node import Node
from serving_robot_interface.action import Arrive
from rclpy.action import ActionClient
import rclpy
class ArrivalRobotKiosk(Node):
    def __init__(self):
        super().__init__('checker')
        self.arithmetic_action_client = ActionClient(
          self,
          Arrive,
          'arrive_robot')

def main(args=None):
    rclpy.init(args=args.argv)
    try:
        checker = Checker()
        checker.send_goal_total_sum(args.goal_total_sum)
        try:
            rclpy.spin(checker)
        except KeyboardInterrupt:
            checker.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            checker.arithmetic_action_client.destroy()
            checker.destroy_node()
    finally:
        rclpy.shutdown()
if __name__ == "__main__":
    main()