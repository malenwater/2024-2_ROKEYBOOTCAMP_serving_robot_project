import sys
import threading
from PyQt5 import QtWidgets, uic, QtCore, QtGui
from PyQt5.QtCore import QRunnable, QThreadPool, pyqtSlot
import rclpy
from rclpy.node import Node
from serving_robot_interface.srv import MySrv
from ..database import data_send
import copy
from ..database import ui_tab

# 테이블 업데이트 작업 클래스
class TableUpdateTask(QRunnable):
    def __init__(self, tables, table_orders):
        super().__init__()
        self.tables = tables
        self.table_orders = table_orders
        self.menu_number = ["","짜장면","간짜장","쟁반짜장",
                "짬뽕","짬뽕밥","짜장밥",
                "탕수육","깐풍기","군만두",
                "팔보채","고추잡채","꽃 빵",
                "사이다","콜라","환타",
                "소주","맥주","고량주",]

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
                    item_menu = QtWidgets.QTableWidgetItem(self.menu_number[menu_index])
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
    reset_signal = QtCore.pyqtSignal() # 리셋 시그널 추가
    def __init__(self, tables):
        super().__init__()
        self.tables = tables
        self.thread_pool = QThreadPool()  # 스레드 풀 생성
        self.table_orders_data = {1 :[], 2 :[], 3 :[],
                                  4 :[], 5 :[], 6 :[],
                                  7 :[], 8 :[], 9 :[],}
    def reset_orders(self):
        print("hihiihihihi11")
        for table_number in self.table_orders_data:
            self.table_orders_data[table_number] = []
        self.update_tables({})
        
    def update_table_orders_data(self,table_orders):
        for table_number in table_orders:
            for count in table_orders[table_number]:
                if len(self.table_orders_data[table_number]):
                    flag = True
                    for menu_order in self.table_orders_data[table_number]:
                        if menu_order[0] == count[0]:
                            menu_order[1] += count[1]
                            flag = False
                    if flag:
                        self.table_orders_data[table_number].append(copy.deepcopy(count))
                else:
                    self.table_orders_data[table_number].append(copy.deepcopy(count))
                
    @QtCore.pyqtSlot(dict)
    def update_tables(self, table_orders):
        # 스레드 풀에서 비동기 작업 실행
        self.update_table_orders_data(table_orders)
        print("aaa",table_orders)
        print("bbb",self.table_orders_data)
        task = TableUpdateTask(self.tables, self.table_orders_data)
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

        # 1) 수신 데이터 파싱 (table_index, menu_index, quantity)
        for i in range(0, len(data), 3):
            try:
                table_index = data[i]
                menu_index = data[i + 1]
                quantity = data[i + 2]
                if table_index not in table_orders:
                    table_orders[table_index] = []
                table_orders[table_index].append([menu_index, quantity])
                print(f"Parsed order -> Table: {table_index}, Menu: {menu_index}, Quantity: {quantity}")
            except IndexError:
                print("Malformed data received")
                response.success = False
                response.message = "Malformed data"
                return response

        # 2) UI 업데이트 (기존과 동일)
        self.ui_updater.update_signal.emit(table_orders)

        # 3) DB 저장 로직 추가 ------------------------------------------
        import datetime
        from ..database import data_send

        data_sender = data_send.DataSender()
        # get_next_order_id()는 MySQLConnector 내부에 있음 → db_connector로 접근
        order_id = data_sender.db_connector.get_next_order_id()
        created_at = updated_at = datetime.datetime.now()

        # table_orders 예: {1: [[4,1], [5,1]]}
        for table_number, products in table_orders.items():
            # (3-1) 먼저 각 테이블별 총가격 계산
            total_price = 0
            for product in products:
                product_id = product[0]
                quantity = product[1]
                price = data_sender.db_connector.get_product_price(product_id)
                total_price += price * quantity
            
            # (3-2) orders 테이블에 INSERT
            order_data = {
                "order_id": order_id,
                "total_price": total_price,
                "table_number": table_number,
                "created_date": created_at,
                "updated_date": updated_at
            }
            if data_sender.insert_order(order_data):
                print(f"주문 정보 삽입 성공 (order_id = {order_id})")
                # (3-3) orders_product 테이블에 INSERT
                for product in products:
                    product_id = product[0]
                    quantity = product[1]
                    price = data_sender.db_connector.get_product_price(product_id)
                    order_product_data = {
                        "order_id": order_id,
                        "product_id": product_id,
                        "quantity": quantity,
                        "price": price,
                        "delivery_completed": 0
                    }
                    if data_sender.insert_order_product(order_product_data):
                        print(f"주문 상품 정보 삽입 성공 (product_id = {product_id})")
                    else:
                        print("주문 상품 정보 삽입 실패")
            else:
                print("주문 정보 삽입 실패")

        data_sender.close()
        # ---------------------------------------------------------------

        response.success = True
        response.message = "Order received and processed (DB insertion done)"
        return response


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
    ui_file = "./src/serving_robot/resource/ui/kit_menu_ui.ui"
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
    robot_widgets = {
        "databaseButton" : dialog.findChild(QtWidgets.QPushButton, "databaseButton"),
        "servingButton" : dialog.findChild(QtWidgets.QPushButton, "servingButton"),
        "turnOFFButton" : dialog.findChild(QtWidgets.QPushButton, "turnOFFButton"),
        "turnONButton" : dialog.findChild(QtWidgets.QPushButton, "turnONButton"),
        "goKittchenButton" : dialog.findChild(QtWidgets.QPushButton, "goKittchenButton"),
        "robot_status" : dialog.findChild(QtWidgets.QLabel, "robot_status"),
    }
    
    # UIUpdater와 Node 인스턴스 생성
    ui_updater = UIUpdater(tables)
    ui_updater.update_signal.connect(ui_updater.update_tables)  # 시그널 연결
    ui_updater.reset_signal.connect(ui_updater.reset_orders)  # 시그널 연결
    node = MyNode(tables, ui_updater)
    
    robot_widgets["databaseButton"].clicked.connect(lambda: handle_databaseButton(dialog))
    robot_widgets["servingButton"].clicked.connect(lambda: handle_servingButton(ui_updater))
    robot_widgets["turnOFFButton"].clicked.connect(lambda: handle_turnOFFButton(robot_widgets))
    robot_widgets["turnONButton"].clicked.connect(lambda: handle_turnONButton(robot_widgets))
    robot_widgets["goKittchenButton"].clicked.connect(handle_goKittchenButton)
    
    for table in tables:
        if table is None:
            print("Error: Table widget not found in UI file.")
        else:
            print(f"{table.objectName()} loaded successfully.")
        table.setVisible(False)



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
    
def handle_databaseButton(dialog):
    node = ui_tab.MainWindow()
    node.exec_()
    
def handle_servingButton(ui_updater):
    print("hi2")
    ui_updater.reset_signal.emit()
    print("hi2")
    pass
def handle_turnOFFButton(robot_widgets):
    print("hi3")
    robot_widgets["robot_status"].setText(str("로봇 상태 : OFF"))
    
    pass
def handle_turnONButton(robot_widgets):
    print("hi4")
    robot_widgets["robot_status"].setText(str("로봇 상태 : ON"))
    pass
def handle_goKittchenButton():
    print("hi5")
    pass
def get_product_price(self, product_id):
        query = "SELECT price FROM products WHERE product_id = %s"
        self.cursor.execute(query, (product_id,))
        result = self.cursor.fetchone()
        if result:
            return result[0]  # 가격 반환
        else:
            print(f"상품 ID {product_id}에 대한 가격을 찾을 수 없습니다.")
            return 0  # 기본값 0 반환


if __name__ == '__main__':
    main()
