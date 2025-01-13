import sys
from PyQt5 import QtWidgets, uic
from playsound import playsound
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from .subscriber import SoundSubscriber
from rclpy.executors import MultiThreadedExecutor
import threading
from .arrival_kiosk import arrival_kiosk
from PyQt5.QtCore import pyqtSignal
#from playsound import playsound

class RobotArrivalDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        # 새로운 UI 파일 로드
        ui_file = "./src/serving_robot/resource/ui/kiosk_arrival.ui"
        try:
            uic.loadUi(ui_file, self)
        except FileNotFoundError:
            print("UI file not found!")
        
        # 알람음(mp3 파일) 경로
        # 버튼 연결
        self.return_robot = self.findChild(QtWidgets.QPushButton, "return_robot")
        self.return_robot.clicked.connect(self.close)  # 버튼 클릭 시 창 닫기
    
    # 알람 재생 함수
 
        
class PayDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        # 새로운 UI 파일 로드
        ui_file = "./src/serving_robot/resource/ui/kiosk_pay.ui"
        try:
            uic.loadUi(ui_file, self)
        except FileNotFoundError:
            print("UI file not found!")
        # 버튼 연결
        self.return_robot = self.findChild(QtWidgets.QPushButton, "return_to_menu")
        self.return_robot.clicked.connect(self.close)  # 버튼 클릭 시 창 닫기

# ROS 2 스레드 클래스
class RosThread(threading.Thread):
    def __init__(self,exc):
        super().__init__()
        self.exc = exc
    def run(self):
        try:
            self.exc.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.node.destroy_node()
            rclpy.shutdown()

class KioskDialog(QtWidgets.QDialog):
    return_robot_start = pyqtSignal()
    return_robot = pyqtSignal(bool)
    return_robot_timeout = pyqtSignal()
    def __init__(self):
        super().__init__()
        # ui 파일 로드
        ui_file = "./src/serving_robot/resource/ui/kiosk.ui"
        self.robot_arrival_dialog = None  # 초기화
        self.return_robot_flag = True
        uic.loadUi(ui_file, self)
        self.table_number = 8
        self.menu_number = ["","짜장면","간짜장","쟁반짜장",
                       "짬뽕","짬뽕밥","짜장밥",
                       "탕수육","깐풍기","군만두",
                       "팔보채","고추잡채","꽃 빵",
                       "사이다","콜라","환타",
                       "소주","맥주","고량주",]
        # 장바구니와 관련된 라벨 정의
        self.total_price = 0
        self.menu_quantities = [0] * 18  # 각 메뉴의 수량
        self.menu_prices = [6000,7000,8000,
                       8000,7000,7000,
                       15000,15000,8000,
                       15000,15000,6000,
                       2000,2000,2000,
                       5000,5000,10000,]  # 각 메뉴의 가격 예시 (각 메뉴별로 다르게 설정 가능)
        self.menu_order = {}
        for i in range(1, 19):  # 1부터 18까지 반복
            self.menu_order[self.menu_number[i]] = str(i)
        # 위젯 딕셔너리 정의
        self.widgets = {
            "foodButton": self.findChild(QtWidgets.QPushButton, "foodButton"),
            "sidefoodButton": self.findChild(QtWidgets.QPushButton, "sidefoodButton"),
            "drinkButton": self.findChild(QtWidgets.QPushButton, "drinkButton"),
            "orders_layout" : self.findChild(QtWidgets.QFormLayout, "orders_layout"),
            "orderButton" : self.findChild(QtWidgets.QPushButton, "orderButton"),
            "total_price" : self.findChild(QtWidgets.QLabel, "total_price"),
            "menu" : self.findChild(QtWidgets.QScrollArea, "menu")
        }
        for i in range(1, 19):
            order_name = self.__make_name_widgets("order_",self.menu_order[self.menu_number[i]])
            select_name = self.__make_name_widgets("selectButton_",self.menu_order[self.menu_number[i]])
            count_name = self.__make_name_widgets("count_",self.menu_order[self.menu_number[i]])
            plusCount_name = self.__make_name_widgets("plusCountButton_",self.menu_order[self.menu_number[i]])
            minusCount_name = self.__make_name_widgets("minusCountButton_",self.menu_order[self.menu_number[i]])
            price_name = self.__make_name_widgets("price_",self.menu_order[self.menu_number[i]])
            self.widgets[select_name] = self.findChild(QtWidgets.QPushButton, f"selectButton_{i}")
            self.widgets[plusCount_name] = self.findChild(QtWidgets.QPushButton, f"plusCountButton_{i}")
            self.widgets[minusCount_name] = self.findChild(QtWidgets.QPushButton, f"minusCountButton_{i}")
            self.widgets[order_name] = self.findChild(QtWidgets.QFrame, order_name)
            self.widgets[price_name] = self.findChild(QtWidgets.QLabel, price_name)
            self.widgets[count_name] = self.findChild(QtWidgets.QLabel, count_name)
            self.widgets[select_name].clicked.connect(lambda _, x=i: self.change_quantity(x, 1))
            self.widgets[plusCount_name].clicked.connect(lambda _, x=i: self.change_quantity(x, 1))
            self.widgets[minusCount_name].clicked.connect(lambda _, x=i: self.change_quantity(x, -1))
            self.widgets["orders_layout"].removeWidget(self.widgets[order_name])
            self.widgets[order_name].setVisible(False)
        self.widgets["foodButton"].clicked.connect(self.__on_scroll_move_button_click)
        self.widgets["sidefoodButton"].clicked.connect(self.__on_scroll_move_button_click)
        self.widgets["drinkButton"].clicked.connect(self.__on_scroll_move_button_click)
        self.widgets["orderButton"].clicked.connect(self.handle_order)
        # 라벨 업데이트 함수
        self.update_labels()
        rclpy.init()
        self.executor = MultiThreadedExecutor()
        self.node = Node('kiosk_node')
        self.publisher = self.node.create_publisher(Int32MultiArray, 'order_data', 10)
        self.sound_sub = SoundSubscriber(self.table_number)
        self.executor.add_node(self.sound_sub)
        # self.return_robot.connect(self.arrival_kiosk.return_robot_signal)
        ros_sound_thread = threading.Thread(target=lambda : self.executor.add_node(self.sound_sub), daemon=True)
        ros_sound_thread.start()

        
        self.return_robot_timeout.connect(self.shutdown_arrive_ui)
        self.return_robot_start.connect(self.arrive_robot)
        self.arrival_kiosk = arrival_kiosk(self.return_robot_timeout, 
                                           self.return_robot_start,
                                           self.table_number)
        self.return_robot.connect(self.arrival_kiosk.return_robot_signal)
        ros_arrive_thread = threading.Thread(target=lambda : self.executor.add_node(self.arrival_kiosk), daemon=True)
        ros_arrive_thread.start()
        
        ros_thread = RosThread(self.executor)
        ros_thread.start()
        print("키오스크 준비 완료")
        
    def shutdown_arrive_ui(self):
        if self.robot_arrival_dialog is not None:
            self.robot_arrival_dialog.close()  # 모달 창 닫기
            self.return_robot_flag = False
        print("로봇 도착창 닫힘")
        
    def __order_menu_widget(self,menu_name):
        self.widgets[menu_name].setVisible(True)
        self.widgets["orders_layout"].addWidget(self.widgets[menu_name])
    def __disorder_menu_widget(self,menu_name):
        self.widgets[menu_name].setVisible(False)
        self.widgets["orders_layout"].removeWidget(self.widgets[menu_name])
    def __make_name_widgets(self, name, widget_number):
        return name + widget_number
    def __on_scroll_move_button_click(self):
        # 버튼 누르면 특정위치 스크롤바 이동
        sender = self.sender()  # 어떤 버튼이 눌렸는지 확인
        scroll_area = self.widgets.get("menu")  # 스크롤 영역 가져오기
        if not scroll_area:
            print("Scroll area not found!")
            return
        scroll_bar = scroll_area.verticalScrollBar()  # 세로 스크롤바 참조
        # 버튼에 따라 스크롤 위치 이동
        if sender == self.widgets["foodButton"]:
            scroll_bar.setValue(0)  # 맨 위로 이동
        elif sender == self.widgets["sidefoodButton"]:
            scroll_bar.setValue(scroll_bar.maximum() // 2)  # 중간으로 이동
        elif sender == self.widgets["drinkButton"]:
            scroll_bar.setValue(scroll_bar.maximum())  # 맨 아래로 이동
    def change_quantity(self, menu_index, change):
        """특정 메뉴의 수량을 증가 또는 감소시킴"""
        # 수량이 0 이상일 때만 감소
        order_name = self.__make_name_widgets("order_",str(menu_index))
        count_name = self.__make_name_widgets("count_",str(menu_index))
        if self.menu_quantities[menu_index - 1] + change >= 0:
            self.menu_quantities[menu_index - 1] += change
            if change == 1 and self.menu_quantities[menu_index - 1] == 1:
                self.__order_menu_widget(order_name)
        if self.menu_quantities[menu_index - 1] == 0:
            self.__disorder_menu_widget(order_name)
        self.widgets[count_name].setText(str(self.menu_quantities[menu_index - 1]))
        self.update_labels()
    def update_labels(self):
        """장바구니 수량과 가격 라벨을 업데이트
            해야할 것: 개수 라벨 업데이트 + 알맞은 가격 업데이트,
        """
        self.total_price = sum(q * p for q, p in zip(self.menu_quantities, self.menu_prices))
        # 장바구니 총 수량, 가격 업데이트
        if self.widgets["total_price"]:
            self.widgets["total_price"].setText(f"총 가격: {self.total_price}원")
        # 각 메뉴별 가격 라벨 업데이트
        for i in range(18):
            price_name = self.__make_name_widgets("price_",self.menu_order[self.menu_number[i+1]])
            if self.widgets[price_name]:
                total_menu_price = self.menu_quantities[i] * self.menu_prices[i]
                self.widgets[price_name].setText(f"{total_menu_price}원")
                
    def reset_order(self):
        for i in range(len(self.menu_quantities)):
            order_name = self.__make_name_widgets("order_",str(i+1))
            if self.menu_quantities[i] != 0:
                self.__disorder_menu_widget(order_name)
            self.menu_quantities[i] = 0  # 각 메뉴의 수량을 0으로 설정
        if self.widgets["total_price"]:
            self.widgets["total_price"].setText(f"총 가격: 0원")
            
    def handle_order(self):
        msg = Int32MultiArray()
        all_data = []

        for menu_name, menu_index_str in self.menu_order.items():
            menu_index = int(menu_index_str)
            quantity = self.menu_quantities[menu_index - 1]
            if quantity > 0:
                all_data.extend([self.table_number, menu_index, quantity])

        if not all_data:
            return

        msg.data = all_data
        print(msg.data)
        self.publisher.publish(msg)
        
        # UI 초기화
        self.reset_order()
        #
        self.pay_orders()
        
        # 결제 처리 관련 로직 추가 가능
    # -------------------------------------------------------------------------------


    def pay_orders(self):
        pay_dialog = PayDialog(self)
        pay_dialog.exec_()  # 모달 창 띄우기
        
    def arrive_robot(self):
        """
        헤야할 것 :어떤 노드 신호를 받기(action), 후에 받은 후로부터 시간을 재서 보내주기, 사용자가 도착완료 버튼 누르면 result 혹은 canceld 상태보내기,
        아마 가능하다면 result가 편할 듯, 일정시간 후에는 무조건 result 보내기, 받았을 때 소리 및 UI 구현
        """
        self.robot_arrival_dialog = RobotArrivalDialog(self)
        self.robot_arrival_dialog.exec_()  # 모달 창 띄우기
        if self.return_robot_flag:
            self.return_robot.emit(True)
        self.return_robot_flag = True
        self.robot_arrival_dialog = None  # 초기화
        pass
    def closeEvent(self, event):
        # 프로그램 종료 시 ROS2 노드 정리
        self.node.destroy_node()
        self.arrival_kiosk.destroy_node()
        rclpy.shutdown()
        super().closeEvent(event)

def main(args=None):
    app = QtWidgets.QApplication(sys.argv)
    dialog = KioskDialog()
    dialog.exec_()
    
if __name__ == '__main__':
    main()