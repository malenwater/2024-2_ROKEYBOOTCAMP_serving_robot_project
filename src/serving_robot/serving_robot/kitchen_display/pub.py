import rclpy
from rclpy.node import Node
from serving_robot_interface.srv import MySrv
from std_msgs.msg import Int32MultiArray

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        # 서비스 클라이언트 설정
        self.client = self.create_client(MySrv, 'order_srv')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            
        # 토픽 구독 설정
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'order_data',
            self.order_callback,
            10)
        self.get_logger().info('test_client 완료')

    def order_callback(self, msg):
        # 키오스크로부터 받은 데이터를 서버로 전송
        self.send_request(msg.data)

    def send_request(self, data):
        if len(data) % 3 != 0:
            self.get_logger().error('Data length must be a multiple of 3')
            return
        
        request = MySrv.Request()
        request.data = data
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Response: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()