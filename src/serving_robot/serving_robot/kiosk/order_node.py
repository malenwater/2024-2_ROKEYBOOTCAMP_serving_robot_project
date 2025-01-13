import rclpy
from rclpy.node import Node
from serving_robot_interface.srv import MySrv  # 서비스 타입 이름 수정
import threading

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.client = self.create_client(MySrv, 'order_srv')  # MySrv로 수정
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, data):
        if len(data) % 3 != 0:
            self.get_logger().error('Data length must be a multiple of 3')
            return

        request = MySrv.Request()  # 서비스 요청 생성
        request.data = data
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Response: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
def ros_thread():
    rclpy.init()
    node = ClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()
    ros_thread_instance = threading.Thread(target=ros_thread)
    ros_thread_instance.start()

    try:
        while rclpy.ok():
            input_str = input("Enter a list of integers (length must be a multiple of 3) (e.g., [1,2,3,4,5,6]): ")
            try:
                data = eval(input_str)
                if len(data) % 3 != 0:
                    print("The list length must be a multiple of 3.")
                    continue
                node.send_request(data)
            except (SyntaxError, ValueError):
                print("Invalid input format. Please enter a valid list of integers.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




