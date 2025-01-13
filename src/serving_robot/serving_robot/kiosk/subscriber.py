import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from playsound import playsound  # 소리 재생을 위한 라이브러리

class SoundSubscriber(Node):
    def __init__(self,table_number):
        super().__init__('sound_subscriber')
        self.subscription = self.create_subscription(
            String,
            'sound_topic',
            self.listener_callback,
            10)
        self.table_number = str(table_number)
        self.subscription
        self.get_logger().info('SoundSubscriber 준비완료')
        print("sound의 테이블 번호",table_number)
    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        if msg.data == self.table_number:
            self.play_sound()

    def play_sound(self):
        # 소리 파일 경로를 설정하고 재생
        playsound('./src/serving_robot/resource/sound/알람음.mp3')  # 실제 경로로 설정

def main(args=None):
    rclpy.init(args=args)
    sound_subscriber = SoundSubscriber()
    rclpy.spin(sound_subscriber)
    sound_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
