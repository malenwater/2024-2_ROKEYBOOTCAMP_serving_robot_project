import sys
from PyQt5 import QtWidgets, uic

class KioskDialog(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()
        # ui 파일 로드
        ui_file = "/mnt/sda1/rokey_project/1week/ROKEY_serving_robot_A-2/src/serving_robot/resource/ui/kiosk.ui"
        uic.loadUi(ui_file, self)
        menu_number = ["","짜장면","간짜장","쟁반짜장",
                       "짬뽕","짬뽕밥","짜장밥",
                       "탕수육","깐풍기","군만두",
                       "팔보채","고추잡채","꽃 빵",
                       "사이다","콜라","환타",
                       "소주","맥주","고량주",]
        menu_order = {}
        for i in range(1, 19):  # 1부터 18까지 반복
            menu_order[menu_number[i]] = str(i)
        # 위젯 딕셔너리 정의
        self.widgets = {
            "foodButton": self.findChild(QtWidgets.QPushButton, "foodButton"),
            "sidefoodButton": self.findChild(QtWidgets.QPushButton, "sidefoodButton"),
            "drinkButton": self.findChild(QtWidgets.QPushButton, "drinkButton"),
            # 주문하기 18개 위젯 만들어서 1개의 함수로 통일 시키기
            # 결제하기 함수 1개
            # -, + 이 둘도 하나의 함수로 만들기
            # 라벨 음식갯수 라벨, 총 가격과 메뉴별 가격 라벨 위젯
            "orders_layout" : self.findChild(QtWidgets.QFormLayout, "orders_layout")
        }
        
        for i in range(1, 19):  # 1부터 18까지 반복
            widget_name = "order_" + menu_order[menu_number[i]]
            self.widgets[widget_name] = self.findChild(QtWidgets.QFrame, widget_name)
            self.widgets["orders_layout"].removeWidget(self.widgets[widget_name]) 
            self.widgets[widget_name].setVisible(False)

        self.widgets["order_1"].setVisible(True)
        self.widgets["order_5"].setVisible(True)
        self.widgets["order_10"].setVisible(True)
        self.widgets["order_18"].setVisible(True)
        self.widgets["order_2"].setVisible(True)
        self.widgets["orders_layout"].addWidget(self.widgets["order_1"]) 
        self.widgets["orders_layout"].addWidget(self.widgets["order_5"]) 
        self.widgets["orders_layout"].addWidget(self.widgets["order_10"]) 
        self.widgets["orders_layout"].addWidget(self.widgets["order_18"]) 
        self.widgets["orders_layout"].addWidget(self.widgets["order_2"]) 
        
        # 버튼에 함수 연결
        self.widgets["foodButton"].clicked.connect(self.on_scroll_move_button_click)
        self.widgets["sidefoodButton"].clicked.connect(self.on_scroll_move_button_click)
        self.widgets["drinkButton"].clicked.connect(self.on_scroll_move_button_click)
       

    def on_scroll_move_button_click(self):
        sender = self.sender()  # 이벤트 발생 위젯 (버튼)
        if sender:
            button_name = sender.objectName()  # 버튼의 이름
            if(button_name == 'foodButton'):
                print("hi")
                # 슬라이더 조종

def main(args=None):
    app = QtWidgets.QApplication(sys.argv)

    # KioskDialog 클래스를 인스턴스화하여 실행
    dialog = KioskDialog()
    dialog.exec_()

if __name__ == '__main__':
    main()
