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
            "label1": self.findChild(QtWidgets.QLabel, "label1"),
            "button1": self.findChild(QtWidgets.QPushButton, "button1")
        }

        # 버튼에 함수 연결
        self.widgets["button1"].clicked.connect(self.on_button_click)

    def on_button_click(self):
        # 버튼 클릭 시 실행될 함수
        self.widgets["label1"].setText("버튼이 클릭되었습니다!")

def main(args=None):
    app = QtWidgets.QApplication(sys.argv)

    # KioskDialog 클래스를 인스턴스화하여 실행
    dialog = KioskDialog()
    dialog.exec_()

if __name__ == '__main__':
    main()
