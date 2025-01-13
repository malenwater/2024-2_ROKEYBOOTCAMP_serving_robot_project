from rclpy.node import Node
from serving_robot_interface.action import Arrive
from rclpy.action import ActionClient
import rclpy
from action_msgs.msg import GoalStatus

class arrival_kitchen(Node):
    def __init__(self,node_name,node_action_name):
        super().__init__(node_name)
        self.arrive_action_client = ActionClient(
          self,
          Arrive,
          node_action_name)
        self.goal_time = 10
        self.get_logger().info('도착 액션 클라이언트 준비 완료')
        
    def send_goal_total_time(self, table_number):
        print("여기")
        while not self.arrive_action_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info('서버 기다리는 중')
        goal_msg = Arrive.Goal()
        goal_msg.goal_time = (float)(self.goal_time)
        goal_msg.table_number = (int)(table_number)
        self.send_goal_future = self.arrive_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.get_arrive_action_feedback)
        self.send_goal_future.add_done_callback(self.get_arrive_action_goal)
        return True

    def get_arrive_action_goal(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Action goal rejected.')
            return
        self.get_logger().info('Action goal accepted.')
        self.action_result_future = goal_handle.get_result_async()
        self.action_result_future.add_done_callback(self.get_arrive_action_result)

    def get_arrive_action_feedback(self, feedback_msg):
        action_feedback = feedback_msg.feedback.rmain_time
        self.get_logger().info('Action feedback: {0}'.format(action_feedback))

    def get_arrive_action_result(self, future):
        action_status = future.result().status
        action_result = future.result().result
        if action_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Action succeeded!')
            self.get_logger().info(
                'Action result(success): {0}'.format(action_result.success))
        else:
            self.get_logger().warning(
                'Action failed with status: {0}'.format(action_status))

def main(args=None):
    rclpy.init()
    table_number = 1
    try:
        arrive_node = arrival_kitchen()
        arrive_node.send_goal_total_time(table_number)
        try:
            rclpy.spin(arrive_node)
        except KeyboardInterrupt:
            arrive_node.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            arrive_node.arrive_action_client.destroy()
            arrive_node.destroy_node()
    finally:
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()