class KioskDialog(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()
        # ui 파일 로드
        ui_file = "/home/gh/바탕화면/ROKEY_serving_robot_A-2/src/serving_robot/resource/ui/kiosk.ui"
        uic.loadUi(ui_file, self)

        # 위젯 딕셔너리 정의
        self.widgets = {
            "foodButton": self.findChild(QtWidgets.QPushButton, "foodButton"),
            "sidefoodButton": self.findChild(QtWidgets.QPushButton, "sidefoodButton"),
            "drinkButton": self.findChild(QtWidgets.QPushButton, "drinkButton"),
            # selectButton_1 부터 selectButton_18까지 정의
            **{f"selectButton_{i}": self.findChild(QtWidgets.QPushButton, f"selectButton_{i}") for i in range(1, 19)},
            # 장바구니 개수 증가/감소 버튼
            **{f"plusCountButton_{i}": self.findChild(QtWidgets.QPushButton, f"plusCountButton_{i}") for i in range(1, 19)},
            **{f"minusCountButton_{i}": self.findChild(QtWidgets.QPushButton, f"minusCountButton_{i}") for i in range(1, 19)},
        }

        # 결제하기 버튼
        self.orderButton = self.findChild(QtWidgets.QPushButton, "orderButton")
        if self.orderButton:
            self.orderButton.clicked.connect(self.handle_order)

        # 장바구니와 관련된 라벨 정의
        self.menu_quantities = [0] * 18  # 각 메뉴의 수량
        self.menu_prices = [1000 + (i * 500) for i in range(18)]  # 각 메뉴의 가격 예시 (각 메뉴별로 다르게 설정 가능)
        self.total_price_label = self.findChild(QtWidgets.QLabel, "total_price")

        # 라벨 업데이트 함수
        self.update_labels()

        # 버튼에 함수 연결
        for i in range(1, 19):
            self.widgets[f"plusCountButton_{i}"].clicked.connect(lambda _, x=i: self.change_quantity(x, 1))
            self.widgets[f"minusCountButton_{i}"].clicked.connect(lambda _, x=i: self.change_quantity(x, -1))

        # 메뉴판 상품 이름 및 가격 라벨 정의 (메뉴판)
        self.menu_labels = {f"label_{i}": self.findChild(QtWidgets.QLabel, f"label_{i}") for i in range(18, 83, 3)}
        self.menu_price_labels = {f"label_{i+1}": self.findChild(QtWidgets.QLabel, f"label_{i+1}") for i in range(18, 83, 3)}

        # 해당 라벨이 없을 경우 기본값 설정
        for i in range(18):
            if not self.menu_labels.get(f"label_{i+18}"):
                self.menu_labels[f"label_{i+18}"] = QtWidgets.QLabel(f"상품 {i+1}")
            if not self.menu_price_labels.get(f"label_{i+19}"):
                self.menu_price_labels[f"label_{i+19}"] = QtWidgets.QLabel(f"{self.menu_prices[i]}원")

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
