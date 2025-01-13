import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SoundPublisher(Node):
    def __init__(self):
        super().__init__('sound_publisher')
        self.publisher_ = self.create_publisher(String, 'sound_topic', 10)
        
    def send_sound_signal(self,table_number):
        msg = String()
        msg.data = str(table_number)  # 메시지를 통해 소리를 재생하도록 지시
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'play_sound'  # 메시지를 통해 소리를 재생하도록 지시
    #     self.publisher_.publish(msg)
    #     self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    sound_publisher = SoundPublisher()
    rclpy.spin(sound_publisher)
    sound_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

