import sys
from PyQt5 import QtWidgets, uic

class KioskDialog(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()
        # ui 파일 로드
        ui_file = "/mnt/sda1/rokey_project/1week/ROKEY_serving_robot_A-2/src/serving_robot/resource/ui/kiosk.ui"
        uic.loadUi(ui_file, self)

        # 위젯 딕셔너리 정의
        self.widgets = {
            "foodButton": self.findChild(QtWidgets.QPushButton, "foodButton"),
            "sidefoodButton": self.findChild(QtWidgets.QPushButton, "sidefoodButton"),
            "drinkButton": self.findChild(QtWidgets.QPushButton, "drinkButton"),
            # 주문하기 18개 위젯 만들어서 1개의 함수로 통일 시키기
            # 결제하기 함수 1개
            # -, + 이 둘도 하나의 함수로 만들기
            # 라벨 음식갯수 라벨, 총 가격과 메뉴별 가격 라벨 위젯
        }

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
