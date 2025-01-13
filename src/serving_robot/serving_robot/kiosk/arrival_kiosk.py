import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from serving_robot_interface.action import Arrive

class arrival_kiosk(Node):
    def __init__(self, return_robot_timeout, return_robot_start,table_number):
        super().__init__('arrival_kiosk_'+str(table_number))
        self.return_robot_timeout = return_robot_timeout
        self.return_robot_start = return_robot_start
        self.table_number =table_number
        self.arithmetic_action_server = ActionServer(
            self,
            Arrive,
            'arrive_robot_'+str(self.table_number), ### 나중에 바꿔야함
            self.execute_checker)
        self.flag = False
        self.get_logger().info('도착 액션 서버 준비 완료')
        print("arrival_kiosk의 테이블 번호",table_number)
    def execute_checker(self, goal_handle):
        self.get_logger().info('Execute arrival action!')
        self.return_robot_start.emit()
        feedback_msg = Arrive.Feedback()
        goal_time = goal_handle.request.goal_time
        feedback_msg.rmain_time = goal_time
        cur_time = 0.0
        while cur_time < goal_time:
            cur_time += 0.1
            feedback_msg.rmain_time = goal_time - cur_time
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.rmain_time))
            goal_handle.publish_feedback(feedback_msg)
            if self.flag:
                break
            time.sleep(0.1)
        goal_handle.succeed()
        result = Arrive.Result()
        result.success = True
        self.flag = False
        self.return_robot_timeout_signal()
        return result
    
    def return_robot_timeout_signal(self):
        self.return_robot_timeout.emit()
        
    def return_robot_signal(self):
        self.flag = True
        
def main(args=None):
    rclpy.init()
    try:
        arrive_node = arrival_kiosk()
        try:
            rclpy.spin(arrive_node)
        except KeyboardInterrupt:
            arrive_node.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            arrive_node.destroy_node()
    finally:
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()