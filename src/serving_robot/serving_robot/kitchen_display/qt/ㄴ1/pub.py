import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(Int32MultiArray, 'order', 10)

    def publish_message(self, label_quantities):
        msg = Int32MultiArray()
        msg.data = label_quantities
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {label_quantities}')

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()

    try:
        while rclpy.ok():
            input_str = input("Enter label index and quantity pairs as a list (e.g., [1,2,3,4]): ")
            try:
                # 입력 받은 문자열을 리스트로 변환
                label_quantities = eval(input_str)
                
                # 리스트의 길이가 짝수인지 확인
                if len(label_quantities) % 2 != 0:
                    print("Please enter pairs of label index and quantity.")
                    continue

                node.publish_message(label_quantities)
            except (SyntaxError, ValueError):
                print("Invalid input format. Please enter a valid list of label index and quantity pairs.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()





