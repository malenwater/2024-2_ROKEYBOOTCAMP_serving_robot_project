import sys
from PyQt5 import QtWidgets, uic
# from playsound import playsound

class RobotArrivalDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        # UI 파일 로드
        ui_file = "./src/serving_robot/resource/ui/kiosk_arrival.ui"
        try:
            uic.loadUi(ui_file, self)
        except FileNotFoundError:
            print("UI file not found!")

        # 버튼 연결
        self.return_robot = self.findChild(QtWidgets.QPushButton, "return_robot")
        if self.return_robot:
            self.return_robot.clicked.connect(self.close_dialog)  # 버튼 클릭 시 창 닫기
        else:
            print("return_robot 버튼을 찾을 수 없습니다.")

        print("here")
        print(self.return_robot.objectName())

    def close_dialog(self):
        print("and hi")
        self.close()  # QDialog의 기본 close 메서드를 호출

class KioskDialog(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()
        # ui 파일 로드
        ui_file = "./src/serving_robot/resource/ui/kiosk.ui"
        uic.loadUi(ui_file, self)
        self.menu_number = ["","짜장면","간짜장","쟁반짜장",
                       "짬뽕","짬뽕밥","짜장밥",
                       "탕수육","깐풍기","군만두",
                       "팔보채","고추잡채","꽃 빵",
                       "사이다","콜라","환타",
                       "소주","맥주","고량주",]
        # 장바구니와 관련된 라벨 정의
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
        total_price = sum(q * p for q, p in zip(self.menu_quantities, self.menu_prices))

        # 장바구니 총 수량, 가격 업데이트
        if self.widgets["total_price"]:
            self.widgets["total_price"].setText(f"총 가격: {total_price}원")

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
        
        #결제하기 버튼 클릭 시 동작
        total_price = sum(q * p for q, p in zip(self.menu_quantities, self.menu_prices))
        print(f"Order confirmed! Total price: {total_price}원")
        
        # 주문 완료 처리: 장바구니 비우기
        self.menu_quantities = [0] * 18
        self.update_labels()  # UI 업데이트

        # 로봇 도착 안내 창 띄우기
        robot_arrival_dialog = RobotArrivalDialog(self)
        robot_arrival_dialog.exec_()  # 모달 창 띄우기

        #해야할 것 : 왼쪽 주문리스트 사라지게 하기, 결제 완료 창 뜨게 하기
        
        total_price = sum(q * p for q, p in zip(self.menu_quantities, self.menu_prices))
        print(f"Order confirmed! Total price: {total_price}원")
        self.reset_order()
        # 결제 처리 관련 로직 추가 가능
        
        # 로봇 도착할때 사용할거 일단 여기에
        # self.alarm_arrive_robot_sound()
        # self.alarm_arrive_robot_ui()
        #
    # -------------------------------------------------------------------------------
    
    def alarm_arrive_robot_sound(self):
            """
            음식 도착 알람을 울리는 함수
            :param file_path: 알람음 파일 경로
            """
            if os.path.exists(file_path):  # 파일이 존재하는지 확인
                try:
                    playsound(file_path)  # 알람음 재생
                except Exception as e:
                    print(f"알람음 재생 중 오류 발생: {e}")
            else:
                print("알람음 파일이 존재하지 않습니다.")
    
    def alarm_arrive_robot_ui(self):
        pass
    def arrive_robot(self):
        """
        헤애힐 것 :어떤 노드 신호를 받기(action), 후에 받은 후로부터 시간을 재서 보내주기, 사용자가 도착완료 버튼 누르면 result 혹은 canceld 상태보내기,
        아마 가능하다면 result가 편할 듯, 일정시간 후에는 무조건 result 보내기, 받았을 때 소리 및 UI 구현
        """
        pass
    
def main(args=None):
    app = QtWidgets.QApplication(sys.argv)
    dialog = KioskDialog()
    dialog.exec_()

if __name__ == '__main__':
    main()
