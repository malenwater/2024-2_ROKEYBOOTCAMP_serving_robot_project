import sys
import os
from PyQt5 import QtWidgets, QtCore, uic
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from PyQt5.uic import QUiLoader

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
            label_index = data[i]
            quantity = data[i + 1]
            if 1 <= label_index <= 27:
                self.labels[label_index - 1].setText(str(quantity))

def load_ui(ui_file, baseinstance):
    loader = QUiLoader()
    file = QtCore.QFile(ui_file)
    file.open(QtCore.QFile.ReadOnly)
    ui = loader.load(file, baseinstance)
    file.close()
    return ui

def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)

    # 절대 경로 설정
    ui_file = "/home/kim/Downloads/qt/menu_ui_초.ui"
    window = QtWidgets.QMainWindow()
    window = load_ui(ui_file, window)

    labels = [
        window.findChild(QtWidgets.QLabel, 'label_3'),
        window.findChild(QtWidgets.QLabel, 'label_7'),
        window.findChild(QtWidgets.QLabel, 'label_9'),
        window.findChild(QtWidgets.QLabel, 'label_31'),
        window.findChild(QtWidgets.QLabel, 'label_33'),
        window.findChild(QtWidgets.QLabel, 'label_35'),
        window.findChild(QtWidgets.QLabel, 'label_39'),
        window.findChild(QtWidgets.QLabel, 'label_41'),
        window.findChild(QtWidgets.QLabel, 'label_43'),
        window.findChild(QtWidgets.QLabel, 'label_55'),
        window.findChild(QtWidgets.QLabel, 'label_57'),
        window.findChild(QtWidgets.QLabel, 'label_59'),
        window.findChild(QtWidgets.QLabel, 'label_63'),
        window.findChild(QtWidgets.QLabel, 'label_65'),
        window.findChild(QtWidgets.QLabel, 'label_67'),
        window.findChild(QtWidgets.QLabel, 'label_71'),
        window.findChild(QtWidgets.QLabel, 'Label_73'),
        window.findChild(QtWidgets.QLabel, 'label_75'),
        window.findChild(QtWidgets.QLabel, 'label_79'),
        window.findChild(QtWidgets.QLabel, 'label_81'),
        window.findChild(QtWidgets.QLabel, 'label_83'),
        window.findChild(QtWidgets.QLabel, 'label_87'),
        window.findChild(QtWidgets.QLabel, 'label_89'),
        window.findChild(QtWidgets.QLabel, 'label_91'),
        window.findChild(QtWidgets.QLabel, 'label_95'),
        window.findChild(QtWidgets.QLabel, 'label_97'),
        window.findChild(QtWidgets.QLabel, 'label_99'),
    ]

    node = MyNode(labels)

    window.show()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
