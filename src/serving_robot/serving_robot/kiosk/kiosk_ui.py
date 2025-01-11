import sys
from PyQt5 import QtWidgets, uic

class KioskDialog(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()
        # ui 파일 로드
        ui_file = "../../resource/ui/kiosk.ui"
        uic.loadUi(ui_file, self)
        self.menu_number = ["","짜장면","간짜장","쟁반짜장",
                       "짬뽕","짬뽕밥","짜장밥",
                       "탕수육","깐풍기","군만두",
                       "팔보채","고추잡채","꽃 빵",
                       "사이다","콜라","환타",
                       "소주","맥주","고량주",]
        self.menu_order = {}
        for i in range(1, 19):  # 1부터 18까지 반복
            self.menu_order[self.menu_number[i]] = str(i)
        # 위젯 딕셔너리 정의
        self.widgets = {
            # 주문하기 18개 위젯 만들어서 1개의 함수로 통일 시키기
            # 결제하기 함수 1개
            # -, + 이 둘도 하나의 함수로 만들기
            # 라벨 음식갯수 라벨, 총 가격과 메뉴별 가격 라벨 위젯
            "foodButton": self.findChild(QtWidgets.QPushButton, "foodButton"),
            "sidefoodButton": self.findChild(QtWidgets.QPushButton, "sidefoodButton"),
            "drinkButton": self.findChild(QtWidgets.QPushButton, "drinkButton"),
            "orders_layout" : self.findChild(QtWidgets.QFormLayout, "orders_layout"),
            "orderButton" : self.findChild(QtWidgets.QPushButton, "orderButton"),
        }
        
        for i in range(1, 19):
            order_name = self.__make_name_widgets("order_",self.menu_order[self.menu_number[i]])
            select_name = self.__make_name_widgets("selectButton_",self.menu_order[self.menu_number[i]])
            plusCount_name = self.__make_name_widgets("plusCountButton_",self.menu_order[self.menu_number[i]])
            minusCount_name = self.__make_name_widgets("minusCountButton_",self.menu_order[self.menu_number[i]])
            price_name = self.__make_name_widgets("price_",self.menu_order[self.menu_number[i]])
            self.widgets[select_name] = self.findChild(QtWidgets.QPushButton, f"selectButton_{i}")
            self.widgets[plusCount_name] = self.findChild(QtWidgets.QPushButton, f"plusCountButton_{i}")
            self.widgets[minusCount_name] = self.findChild(QtWidgets.QPushButton, f"minusCountButton_{i}")
            self.widgets[order_name] = self.findChild(QtWidgets.QFrame, order_name)
            self.widgets[price_name] = self.findChild(QtWidgets.QLabel, price_name)
            self.widgets[plusCount_name].clicked.connect(lambda _, x=i: self.change_quantity(x, 1))
            self.widgets[minusCount_name].clicked.connect(lambda _, x=i: self.change_quantity(x, -1))
            self.widgets["orders_layout"].removeWidget(self.widgets[order_name]) 
            self.widgets[order_name].setVisible(False)
            
        self.widgets["foodButton"].clicked.connect(self.__on_scroll_move_button_click)
        self.widgets["sidefoodButton"].clicked.connect(self.__on_scroll_move_button_click)
        self.widgets["drinkButton"].clicked.connect(self.__on_scroll_move_button_click)
        self.widgets["orderButton"].clicked.connect(self.handle_order)
        # self.__order_menu_widget('order_10')
        # self.__disorder_menu_widget('order_2')
        
        # 장바구니와 관련된 라벨 정의
        self.menu_quantities = [0] * 18  # 각 메뉴의 수량
        self.menu_prices = [1000 + (i * 500) for i in range(18)]  # 각 메뉴의 가격 예시 (각 메뉴별로 다르게 설정 가능)
        self.total_price_label = self.findChild(QtWidgets.QLabel, "total_price")

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
        pass
    
    def change_quantity(self, menu_index, change):
        """특정 메뉴의 수량을 증가 또는 감소시킴"""
        # 수량이 0 이상일 때만 감소
        if self.menu_quantities[menu_index - 1] + change >= 0:
            self.menu_quantities[menu_index - 1] += change
        self.update_labels()

    def update_labels(self):
        """장바구니 수량과 가격 라벨을 업데이트"""
        total_quantity = sum(self.menu_quantities)
        total_price = sum(q * p for q, p in zip(self.menu_quantities, self.menu_prices))

        # 장바구니 총 수량, 가격 업데이트
        if self.total_price_label:
            self.total_price_label.setText(f"총 가격: {total_price}원")

        # 각 메뉴별 가격 라벨 업데이트
        for i in range(18):
            menu_price_label = self.findChild(QtWidgets.QLabel, f"price_{i+1}")
            if menu_price_label:
                total_menu_price = self.menu_quantities[i] * self.menu_prices[i]
                menu_price_label.setText(f"{total_menu_price}원")

        # 메뉴판에서 상품 이름과 가격 업데이트
        for i, label in enumerate(self.menu_labels.values()):
            if label:
                label.setText(f"상품 {i + 1}")

        for i, label in enumerate(self.menu_price_labels.values()):
            if label:
                label.setText(f"{self.menu_prices[i]}원")

    def handle_order(self):
        """결제하기 버튼 클릭 시 동작"""
        total_price = sum(q * p for q, p in zip(self.menu_quantities, self.menu_prices))
        print(f"Order confirmed! Total price: {total_price}원")
        # 결제 처리 관련 로직 추가 가능

def main(args=None):
    app = QtWidgets.QApplication(sys.argv)
    dialog = KioskDialog()
    dialog.exec_()

if __name__ == '__main__':
    main()
