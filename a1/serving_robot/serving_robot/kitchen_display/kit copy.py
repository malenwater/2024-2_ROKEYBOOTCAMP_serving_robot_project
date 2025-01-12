import sys
import threading
from PyQt5 import QtWidgets, uic, QtCore, QtGui
from PyQt5.QtCore import QRunnable, QThreadPool, pyqtSlot
import rclpy
from rclpy.node import Node
from ros_study_msgs2.srv import MySrv

# 테이블 업데이트 작업 클래스
class TableUpdateTask(QRunnable):
    def __init__(self, tables, table_orders):
        super().__init__()
        self.tables = tables
        self.table_orders = table_orders

    @pyqtSlot()
    def run(self):
        for idx, table in enumerate(self.tables):
            table_index = idx + 1
            if table_index in self.table_orders:
                orders = self.table_orders[table_index]
                table.setRowCount(len(orders))
                table.setColumnCount(2)
                table.setVisible(True)
                for row, (menu_index, quantity) in enumerate(orders):
                    item_menu = QtWidgets.QTableWidgetItem(str(menu_index))
                    item_quantity = QtWidgets.QTableWidgetItem(str(quantity))
                    item_menu.setForeground(QtGui.QColor(255, 255, 255))  # 흰색 텍스트
                    item_quantity.setForeground(QtGui.QColor(255, 255, 255))  # 흰색 텍스트
                    table.setItem(row, 0, item_menu)
                    table.setItem(row, 1, item_quantity)
            else:
                table.setRowCount(0)
                table.setVisible(False)

# UI 업데이트 클래스
class UIUpdater(QtCore.QObject):
    update_signal = QtCore.pyqtSignal(dict)

    def __init__(self, tables):
        super().__init__()
        self.tables = tables
        self.thread_pool = QThreadPool()  # 스레드 풀 생성

    @QtCore.pyqtSlot(dict)
    def update_tables(self, table_orders):
        # 스레드 풀에서 비동기 작업 실행
        task = TableUpdateTask(self.tables, table_orders)
        self.thread_pool.start(task)

# ROS 2 노드 클래스
class MyNode(Node):
    def __init__(self, tables, ui_updater):
        super().__init__('kitchen_node')
        self.tables = tables
        self.ui_updater = ui_updater
        self.srv = self.create_service(MySrv, 'order_srv', self.service_callback)
        print("Service 'order_srv' created and waiting for requests...")

    def service_callback(self, request, response):
        data = request.data
        table_orders = {}

        if not data:
            print("Received empty data")
            response.success = False
            response.message = "No data received"
            return response

        for i in range(0, len(data), 3):
            try:
                table_index = data[i]
                menu_index = data[i + 1]
                quantity = data[i + 2]
                if table_index not in table_orders:
                    table_orders[table_index] = []
                table_orders[table_index].append((menu_index, quantity))
                print(f"Parsed order -> Table: {table_index}, Menu: {menu_index}, Quantity: {quantity}")
            except IndexError:
                print("Malformed data received")
                response.success = False
                response.message = "Malformed data"
                return response

        # UI 업데이트 요청을 시그널을 통해 보냄
        self.ui_updater.update_signal.emit(table_orders)

        response.success = True
        response.message = "Order received and processed"
        return response

    def clear_tables(self):
        for table in self.tables:
            table.setRowCount(0)
            table.setVisible(False)
        self.get_logger().info("Tables cleared.")

# ROS 2 스레드 클래스
class RosThread(threading.Thread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        finally:
            self.node.destroy_node()
            rclpy.shutdown()

# 메인 함수
def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)

    # UI 파일 로드
    ui_file = "/home/kim/다운로드/qt/kit_menu_ui.ui"
    dialog = QtWidgets.QDialog()
    uic.loadUi(ui_file, dialog)

    # 테이블 위젯 가져오기
    tables = [
        dialog.findChild(QtWidgets.QTableWidget, 'tableWidget_2'),
        dialog.findChild(QtWidgets.QTableWidget, 'tableWidget_3'),
        dialog.findChild(QtWidgets.QTableWidget, 'tableWidget_4'),
        dialog.findChild(QtWidgets.QTableWidget, 'tableWidget_5'),
        dialog.findChild(QtWidgets.QTableWidget, 'tableWidget_6'),
        dialog.findChild(QtWidgets.QTableWidget, 'tableWidget_7'),
        dialog.findChild(QtWidgets.QTableWidget, 'tableWidget_8'),
        dialog.findChild(QtWidgets.QTableWidget, 'tableWidget_9'),
        dialog.findChild(QtWidgets.QTableWidget, 'tableWidget_10'),
    ]

    for table in tables:
        if table is None:
            print("Error: Table widget not found in UI file.")
        else:
            print(f"{table.objectName()} loaded successfully.")
        table.setVisible(False)

    # UIUpdater와 Node 인스턴스 생성
    ui_updater = UIUpdater(tables)
    ui_updater.update_signal.connect(ui_updater.update_tables)  # 시그널 연결
    node = MyNode(tables, ui_updater)

    # 버튼 클릭 시 테이블 초기화
    complete_button = dialog.findChild(QtWidgets.QPushButton, 'pushButton_7')
    if complete_button is not None:
        complete_button.clicked.connect(node.clear_tables)
        print("Button 'pushButton_7' connected successfully.")
    else:
        print("Error: pushButton_7 not found in UI file.")

    # ROS2 스레드 실행
    ros_thread = RosThread(node)
    ros_thread.start()

    # UI 실행
    dialog.exec_()

    # UI 종료 시 ROS2 스레드 정리
    ros_thread.join()

if __name__ == '__main__':
    main()
