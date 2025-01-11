import sys
from PyQt5 import QtWidgets, uic
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class MyNode(Node):
    def __init__(self, labels):
        super().__init__('my_node')
        self.labels = labels
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'order',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        data = msg.data
        for i in range(0, len(data), 2):
            print(f'Received: {data}')
            label_index = data[i]
            quantity = data[i + 1]
            if 1 <= label_index <= 27:
                self.labels[label_index - 1].setText(str(quantity))

def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)

    # 절대 경로를 명확히 지정
    ui_file = "/home/kim/Downloads/qt/주방_menu_ui_초.ui"
    dialog = QtWidgets.QDialog()
    uic.loadUi(ui_file, dialog)

    labels = [
        dialog.findChild(QtWidgets.QLabel, 'label_3'),
        dialog.findChild(QtWidgets.QLabel, 'label_7'),
        dialog.findChild(QtWidgets.QLabel, 'label_9'),
        dialog.findChild(QtWidgets.QLabel, 'label_31'),
        dialog.findChild(QtWidgets.QLabel, 'label_33'),
        dialog.findChild(QtWidgets.QLabel, 'label_35'),
        dialog.findChild(QtWidgets.QLabel, 'label_39'),
        dialog.findChild(QtWidgets.QLabel, 'label_41'),
        dialog.findChild(QtWidgets.QLabel, 'label_43'),
        dialog.findChild(QtWidgets.QLabel, 'label_55'),
        dialog.findChild(QtWidgets.QLabel, 'label_57'),
        dialog.findChild(QtWidgets.QLabel, 'label_59'),
        dialog.findChild(QtWidgets.QLabel, 'label_63'),
        dialog.findChild(QtWidgets.QLabel, 'label_65'),
        dialog.findChild(QtWidgets.QLabel, 'label_67'),
        dialog.findChild(QtWidgets.QLabel, 'label_71'),
        dialog.findChild(QtWidgets.QLabel, 'Label_73'),
        dialog.findChild(QtWidgets.QLabel, 'label_75'),
        dialog.findChild(QtWidgets.QLabel, 'label_79'),
        dialog.findChild(QtWidgets.QLabel, 'label_81'),
        dialog.findChild(QtWidgets.QLabel, 'label_83'),
        dialog.findChild(QtWidgets.QLabel, 'label_87'),
        dialog.findChild(QtWidgets.QLabel, 'label_89'),
        dialog.findChild(QtWidgets.QLabel, 'label_91'),
        dialog.findChild(QtWidgets.QLabel, 'label_95'),
        dialog.findChild(QtWidgets.QLabel, 'label_97'),
        dialog.findChild(QtWidgets.QLabel, 'label_99'),
    ]

    node = MyNode(labels)

    dialog.exec_()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
