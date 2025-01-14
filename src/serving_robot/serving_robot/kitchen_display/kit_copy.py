import sys
import threading
import ctypes
from PyQt5 import QtWidgets, uic, QtCore, QtGui
from PyQt5.QtCore import QRunnable, QThreadPool, pyqtSlot
import rclpy
from rclpy.node import Node
from serving_robot_interface.srv import MySrv
from ..database import data_send
import copy
from ..database import ui_tab
from ..kiosk.publisher import SoundPublisher
from std_msgs.msg import Int32
from ..kitchen_display.arrival_kitchen import arrival_kitchen
from rclpy.executors import MultiThreadedExecutor
import asyncio
import time
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
    go_table_by_path = QtCore.pyqtSignal() 
    check_arrive_robot_signal = QtCore.pyqtSignal() 
    check_goal_total_time_signal = QtCore.pyqtSignal()
    def __init__(self, tables, node,_arrival_kitchens):
        super().__init__()
        self.tables = tables
        self.node = node  # MyNode 인스턴스를 저장
        self._arrival_kitchens = _arrival_kitchens
        self.scheduler_thread_10 = None
        self.thread_pool = QThreadPool()  # 스레드 풀 생성
        self.table_orders_data = {1 :[], 2 :[], 3 :[],
                                  4 :[], 5 :[], 6 :[],
                                  7 :[], 8 :[], 9 :[],}
        self.road_table_go = []
        self.TurnON_flag = True
        self.current_serving_status = False
        self.road_table_go_data_save = {}
        self.check_arrive_robot = False
        self.check_goal_total_time = False
        self.scheduler_thread = None
        self.check_arrive_robot_signal.connect(self.change_staus)
        self.check_goal_total_time_signal.connect(self.change_staus_kiosk)
    def reset_orders(self):
        for table_number in self.table_orders_data:
            self.table_orders_data[table_number] = []
        self.update_tables({})
    '''
    # 해야할 것, 현재 어떤 테이블에 값이 있는가? 이를 저장하고
    # 이에 따라 이동을 한다.
    # 이동 결과가 나오면 내가 만든 함수를 호출 하고 결과 값을 리턴 받는다. playsound 또한 보낸다.
    # 비어있는지 확인하고 비어있으면 주방으로 아니면 똑같이 반복한다.
    
    # 또한 사용자가 서빙 중 인데 키를 누른다면
    # 주방서빙하는 키 = 동작 중이므로 무시한다. 이를 띄워야한다... 
    # 멈추는 키 = 일단은 리셋상태 유지, 다시 켜도 이동하지 않고, 주방 돌아오기를 수행
    # 전원 키는 키 = 그냥 동작한다. 굳이 구현 안 해도 됨                                        완료
    # 주방 돌아오는 키 = 무시한다.
    # 일단 쭉 이동하는거 구현하기
    '''
    def change_staus(self):
        self.check_arrive_robot = True
    def change_staus_kiosk(self):
        self.check_goal_total_time = True
    def start_scheduler_thread(self):
        # scheduler_robot_go_table 함수를 별도의 스레드로 실행
        self.road_table_go = [key for key, value in self.table_orders_data.items() if value]
        if self.current_serving_status or len(self.road_table_go) == 0:
            return
        self.scheduler_thread = threading.Thread(target=self.scheduler_robot_go_table, daemon=True)
        self.scheduler_thread.start()
        self.current_serving_status = True
        print("경로 찾기 쓰레드 시작")
         
    def scheduler_robot_go_table(self):
        print("경로 시작")
        self.road_table_go_data_save = copy.deepcopy(self.table_orders_data)
        print(self.road_table_go)
        print(self.road_table_go_data_save)
        print("현재 가야하는 경로 스케줄")
        for table_number in self.road_table_go:
            self.node.send_target_number(table_number) # check_arrive_robot = 
            print("자러간다.")
            while self.check_arrive_robot != True:
                time.sleep(0.1)
                print(self.check_arrive_robot)
            print("깨어났다.. 제어 2개를 하고 값을 받을 때까지 대기한다.")
            sound = SoundPublisher()
            sound.send_sound_signal(table_number)
            self._arrival_kitchens[table_number].send_goal_total_time(10)
            while self.check_goal_total_time != True:
                time.sleep(0.1)
                # time.sleep(3)
                # print(self.check_goal_total_time)
            self.check_arrive_robot = False
            self.check_goal_total_time = False
            print("깨어났다.. 다시 반복한다.")
            
        print("하나씩 이동, 이동 완료시 값을 받는다. 키오스크에 보낸다. 2개 소리와 action, 또 값을 받아야 다음 실행을 한다. 이를 반복한다.")
        self.node.send_target_number(10)
        while self.check_arrive_robot != True:
            time.sleep(0.1)
        self.check_arrive_robot = False
        print("주방으로 이동")
        print("경로 완료")
        self.current_serving_status = False
        
        pass
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
        print("저장확인",self.road_table_go_data_save)
        task = TableUpdateTask(self.tables, self.table_orders_data)
        self.thread_pool.start(task)

# ROS 2 노드 클래스
class MyNode(Node):
    def __init__(self, tables, ui_updater):
        super().__init__('kitchen_node')
        self.tables = tables
        self.ui_updater = ui_updater
        self.expected_sequence = []  # 서비스에서 받은 테이블 번호 순서
        self.received_sequence = []  # 실제 구독한 테이블 번호 순서

        # self.check_arrive_robot_to_table = False
        self.srv = self.create_service(MySrv, 'order_srv', self.service_callback)
        self.subscription = self.create_subscription(Int32, 'arrival_notification', self.notification_callback, 10) # 로봇에서 테이블 번호 구독
        
        #주방에서 로봇 토픽 발행을 QOS로 설정
        self.publisher = self.create_publisher(
    Int32,
    'table',
    qos_profile=rclpy.qos.QoSProfile(
        reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,  # 반드시 메시지 전달
        history=rclpy.qos.HistoryPolicy.KEEP_LAST,  # 최신 명령 1개 유지
        depth=1
    )
)

        print("Service 'order_srv' created and waiting for requests...")

    def service_callback(self, request, response):
        data = request.data
        table_orders = {}

        # 로깅용 
        self.expected_sequence = [data[i] for i in range(0, len(data), 3)]  # 3의 배수 인덱스로 테이블 번호 추출
        self.received_sequence = []  # 새로운 수신 시 초기화
        
        
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
    def notification_callback(self, msg):
        """구독자 콜백 함수: 도착 알림 메시지 수신 시 출력"""
        table_number = msg.data
        self.received_sequence.append(table_number)
        self.get_logger().info(f"Received arrival notification: Table {table_number}")
        self.ui_updater.check_arrive_robot_signal.emit()
        print("신호를 보내긴 보냄")
        if not (1 <= table_number <= 12):
            self.get_logger().warn(f"Invalid table number received: {table_number}")
            return
        current_index = len(self.received_sequence) - 1
        if current_index < len(self.expected_sequence):
            expected_number = self.expected_sequence[current_index]
            if table_number == expected_number:
                self.get_logger().info(f"Correct table number received: {table_number}")
            else:
                self.get_logger().warn(f"Out of order: expected {expected_number}, but received {table_number}")
        else:
            self.get_logger().warn(f"Extra table number received: {table_number}")

        # 모든 테이블 번호 수신 여부 확인
        if len(self.received_sequence) == len(self.expected_sequence):
            if self.received_sequence == self.expected_sequence:
                self.get_logger().info(f"All table numbers received in correct order: {self.received_sequence}")
            else:
                self.get_logger().warn(f"Final order out of sequence: {self.received_sequence}")
            self.received_sequence.clear()

        # 수신 개수 확인
        if len(self.received_sequence) > len(self.expected_sequence):
            self.get_logger().error(f"Too many table numbers received: {self.received_sequence}")
            self.received_sequence.clear()
        elif len(self.received_sequence) < len(self.expected_sequence) and len(self.received_sequence) == len(self.expected_sequence) - 1:
            self.get_logger().error("Too few table numbers received.")




        # self.check_arrive_robot_to_table = True
        # print("도착했다.",self.check_arrive_robot_to_table)



    
    # 퍼블리시 메소드 (스레드 처리 완료)
    def send_target_number(self, number):
        """발행 함수: 버튼 클릭 시 발행할 숫자를 ROS2 토픽으로 발행"""
        msg = Int32()
        msg.data = number  # 발행할 데이터 (버튼에 따라 11, 12, 0)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published target number: {number}') 
        # self.check_arrive_robot_to_table = False
        # return self.check_arrive_robot_to_table



# ROS 2 스레드 클래스
class RosThread(threading.Thread):
    def __init__(self, node,exc):
        super().__init__()
        self.node = node
        self.exc = exc
    def run(self):
        try:
            self.exc.add_node(self.node)
            # rclpy.spin(self.node)
            self.exc.spin()
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
    ui_updater = UIUpdater(tables, None, None)
    ui_updater.update_signal.connect(ui_updater.update_tables)  # 시그널 연결
    ui_updater.go_table_by_path.connect(ui_updater.start_scheduler_thread)  # 시그널 연결
    ui_updater.reset_signal.connect(ui_updater.reset_orders)  # 시그널 연결
    node = MyNode(tables, ui_updater)
    ui_updater.node = node 
    robot_widgets["databaseButton"].clicked.connect(lambda: handle_databaseButton(dialog))
    robot_widgets["servingButton"].clicked.connect(lambda: handle_servingButton(ui_updater))
    robot_widgets["turnOFFButton"].clicked.connect(lambda: handle_turnOFFButton(robot_widgets,node,ui_updater))
    robot_widgets["turnONButton"].clicked.connect(lambda: handle_turnONButton(robot_widgets,node,ui_updater))
    robot_widgets["goKittchenButton"].clicked.connect(lambda: handle_goKittchenButton(node,ui_updater))

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
    executor = MultiThreadedExecutor()
    _arrival_kitchens ={}
    for idx in range(1,10):
        print("i",idx)
        node_name = "arrival_kitchen_" + str(idx)
        node_action_name = "arrive_robot_" + str(idx)
        _arrival_kitchen = arrival_kitchen(node_name,node_action_name,ui_updater)
        ros_arrive_thread = threading.Thread(target=lambda : executor.add_node(_arrival_kitchen), daemon=True)
        ros_arrive_thread.start()
        _arrival_kitchens[idx] = _arrival_kitchen
    ui_updater._arrival_kitchens = _arrival_kitchens 
    
    # ROS2 스레드 실행
    ros_thread = RosThread(node,executor)
    ros_thread.start()

    # UI 실행
    dialog.exec_()

    # UI 종료 시 ROS2 스레드 정리
    ros_thread.join()
    
def handle_databaseButton(dialog):
    node = ui_tab.MainWindow()
    node.exec_()
    
def handle_servingButton(ui_updater):
    if ui_updater.current_serving_status != True and ui_updater.TurnON_flag:
        ui_updater.go_table_by_path.emit()
        if len(ui_updater.road_table_go) != 0:
            ui_updater.reset_signal.emit()

def handle_turnOFFButton(robot_widgets,node,ui_updater):
    robot_widgets["robot_status"].setText(str("로봇 상태 : OFF"))
    node.send_target_number(11)
    ui_updater.check_arrive_robot = False
    ui_updater.current_serving_status = False
    ui_updater.check_goal_total_time = False
    ui_updater.TurnON_flag = False    
    try:
        terminate_thread(ui_updater.scheduler_thread)
        print("1~9 스레드가 종료되었습니다.")
        ui_updater.scheduler_thread = None
    except SystemError as e:
        print("SystemError 발생:", e)
    except Exception as e:
        print("예상치 못한 오류 발생:", e)
    try:
        terminate_thread(ui_updater.scheduler_thread_10)
        print("10 스레드가 종료되었습니다.")
        ui_updater.scheduler_thread_10 = None
    except SystemError as e:
        print("SystemError 발생:", e)
    except Exception as e:
        print("예상치 못한 오류 발생:", e)
    pass

def handle_turnONButton(robot_widgets,node,ui_updater):
    robot_widgets["robot_status"].setText(str("로봇 상태 : ON"))
    node.send_target_number(12)
    ui_updater.check_arrive_robot = False
    ui_updater.TurnON_flag = True    
    pass

def handle_goKittchenButton(node,ui_updater):
    def go_kitchen(ui_updater):
        while ui_updater.check_arrive_robot != True:
            time.sleep(0.1)
        ui_updater.check_arrive_robot = False
        ui_updater.current_serving_status = False
        
     
    if ui_updater.current_serving_status != True and ui_updater.TurnON_flag:
        ui_updater.current_serving_status = True
        node.send_target_number(10)
        scheduler_thread = threading.Thread(target=go_kitchen, args=(ui_updater,), daemon=True)
        scheduler_thread.start()
        ui_updater.scheduler_thread_10 = scheduler_thread


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
def terminate_thread(thread):
    if not thread.is_alive():
        return
    exc = ctypes.py_object(SystemExit)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
        ctypes.c_long(thread.ident), exc
    )
    if res == 0:
        raise ValueError("스레드 ID를 찾을 수 없습니다.")
    elif res > 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(thread.ident, None)
        raise SystemError("스레드 종료에 실패했습니다.")

if __name__ == '__main__':
    main()
