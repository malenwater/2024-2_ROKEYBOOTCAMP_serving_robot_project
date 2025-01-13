import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class TargetNumberPublisher(Node):
    def __init__(self):
        super().__init__('target_number_publisher')
        # 퍼블리셔 생성
        self.publisher = self.create_publisher(Int32, 'table', 10)

    def send_target_number(self, number):
        msg = Int32()
        msg.data = number  # 전송할 목표 번호
        self.publisher.publish(msg)
        self.get_logger().info(f'Published target number: {number}')

def main():
    rclpy.init()
    node = TargetNumberPublisher()

    try:
        while rclpy.ok():
            user_input = input("Enter target number (1-12): ").strip()  # 사용자 입력 후 공백 제거

            # **빈 입력 방지**
            if not user_input:
                print("Error: Input cannot be empty. Please enter a valid number.")
                continue  # 루프 재시작

            try:
                number = int(user_input)  # 입력값을 정수로 변환
                if 1 <= number <= 12:
                    node.send_target_number(number)
                else:
                    print("Error: Please enter a valid number between 1 and 12.")
            except ValueError:
                print("Error: Invalid input. Please enter a valid number.")

    except KeyboardInterrupt:
        print("\nShutting down...")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


