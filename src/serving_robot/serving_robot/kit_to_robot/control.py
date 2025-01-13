import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Int32
import time

class NavigationSubscriber(Node):
    def __init__(self):
        super().__init__('navigation_subscriber')
        self.subscription = self.create_subscription(Int32, 'table', self.number_callback, 10)
        self.publisher = self.create_publisher(Int32, 'arrival_notification', 10)  # 발행자 추가
        self.navigate_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_coordinates = [
            (1.90, 0.67), (0.72, 1.0), (0.72, 0), (0.72, -1),
            (-0.33, 1.0), (-0.33, 0), (-0.33, -1), (-1.4, 1), (-1.4, 0), (-1.4, -1)
        ]
        self.is_paused = False  # 일시 정지 상태 여부
        self.current_goal_handle = None  # 현재 액션 핸들 저장
        self.send_goal_future = None  # 액션 목표 전송 결과 저장
        self.current_target_number = None  # 목표 번호 초기화

    def number_callback(self, msg):
        number = msg.data
        self.get_logger().info(f'Received target number: {number}')

        if number == 11:
            self.is_paused = True  # 일시 정지 설정
            self.cancel_current_goal()
            self.get_logger().info('Paused: Robot stopped and commands ignored.')

        elif number == 12:
            self.is_paused = False  # 재개 설정
            self.reconnect_to_action_server()  # 액션 클라이언트를 새로 생성
            self.current_goal_handle = None  # 현재 goal 상태 초기화
            self.get_logger().info('Resumed: Robot will now accept navigation goals.')

        elif 0 <= number <= 9:
            if self.is_paused:
                self.get_logger().warn(f'Received target number {number}, but the robot is paused. Ignoring command.')
            else:
                map_x, map_y = self.goal_coordinates[number]
                self.current_target_number = number  # 목표 번호 저장
                self.send_goal(map_x, map_y)

    def send_goal(self, map_x, map_y):
        if not self.navigate_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('NavigateToPose action server not available.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = float(map_x)
        goal_msg.pose.pose.position.y = float(map_y)
        goal_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.get_logger().info(f'Sending goal: x={goal_msg.pose.pose.position.x}, y={goal_msg.pose.pose.position.y}')
        self.send_goal_future = self.navigate_action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected.')
            return
        self.get_logger().info('Goal accepted. Navigating...')
        self.current_goal_handle = goal_handle
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result.status == 4:
            self.get_logger().info('Navigation succeeded!')
            self.publish_arrival_notification()  # 도착 알림 발행
        else:
            self.get_logger().warn(f'Navigation failed with status: {result.status}')

    def publish_arrival_notification(self):
        """목표 위치에 도착했을 때 숫자를 발행"""
        if self.current_target_number is not None:
            msg = Int32()
            msg.data = self.current_target_number
            self.publisher.publish(msg)
            self.get_logger().info(f'Published arrival notification: {self.current_target_number}')
        else:
            self.get_logger().warn('No target number available to publish.')

    def cancel_current_goal(self):
        if self.current_goal_handle is not None:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            time.sleep(0.5)
        else:
            self.get_logger().info('No active goal to cancel.')

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if cancel_response.return_code == 0:  # 0: 취소 성공
            self.get_logger().info('Goal cancel request accepted.')
            self.current_goal_handle = None
        else:
            self.get_logger().warn('Goal cancel request was rejected.')

    def reconnect_to_action_server(self):
        self.get_logger().info('Reconnecting to action server...')
        self.navigate_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        if not self.navigate_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Failed to reconnect to the action server.')
        else:
            self.get_logger().info('Successfully reconnected to the action server.')

def main():
    rclpy.init()
    node = NavigationSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





