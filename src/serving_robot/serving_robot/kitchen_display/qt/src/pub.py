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
            label_quantities = []
            while True:
                label_index = int(input("Enter label index (1-27), or 0 to stop: "))
                if label_index == 0:
                    break
                quantity = int(input("Enter quantity: "))
                label_quantities.append(label_index)
                label_quantities.append(quantity)
            
            if label_quantities:
                node.publish_message(label_quantities)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

