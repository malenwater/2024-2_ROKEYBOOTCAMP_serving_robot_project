import sys
from PyQt5.QtWidgets import QApplication, QDialog, QPushButton, QVBoxLayout, QLabel ,QListWidget
from PyQt5.uic import loadUi
import random
from PyQt5.QtCore import Qt
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

class PopupDialog1(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('일일 매출')
        self.setGeometry(100, 100, 800, 600)

        # 레이아웃 설정
        layout = QVBoxLayout()
        self.canvas = FigureCanvas(plt.figure(figsize=(8, 6)))  # matplotlib 캔버스
        layout.addWidget(self.canvas)

        self.setLayout(layout)
        
        # 임의로 매출 데이터 생성
        self.generate_random_sales_data()

    def generate_random_sales_data(self):
        # 임의의 날짜 리스트 (예: 1월 1일부터 7일까지)
        dates = ['2025-01-0' + str(i) for i in range(1, 8)]
        
        # 임의로 매출 데이터 생성 (1000에서 5000 사이의 랜덤값)
        sales = [random.randint(1000, 5000) for _ in dates]

        # 그래프 그리기
        ax = self.canvas.figure.add_subplot(111)
        ax.clear()
        ax.plot(dates, sales, marker='o', linestyle='-', color='b')
        ax.set_title("일일 매출")
        ax.set_xlabel("날짜")
        ax.set_ylabel("매출")
        ax.grid(True)
        self.canvas.draw()  # 그래프 갱신



class PopupDialog2(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('메뉴별 매출')
        self.setGeometry(100, 100, 800, 600)

        # 레이아웃 설정
        layout = QVBoxLayout()
        self.canvas = FigureCanvas(plt.figure(figsize=(8, 6)))  # matplotlib 캔버스
        layout.addWidget(self.canvas)

        self.setLayout(layout)
        
        # 메뉴별 매출 데이터 생성
        self.generate_menu_sales_data()

    def generate_menu_sales_data(self):
        # 메뉴 리스트
        menu_items = ['짜장면', '짬뽕', '탕수육']
        
        # 각 메뉴에 대한 임의 매출 데이터 생성 (100~1000 사이의 랜덤값)
        sales = [random.randint(100, 1000) for _ in menu_items]

        # 그래프 그리기
        ax = self.canvas.figure.add_subplot(111)
        ax.clear()
        ax.bar(menu_items, sales, color=['orange', 'green', 'blue'])
        ax.set_title("메뉴별 매출")
        ax.set_xlabel("메뉴")
        ax.set_ylabel("매출")
        ax.grid(True, axis='y')
        self.canvas.draw()  # 그래프 갱신

class PopupDialog3(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('추천 메뉴')
        self.setGeometry(100, 100, 400, 300)  # 윈도우 크기 조정

        # 레이아웃 설정
        layout = QVBoxLayout()

        # 추천 메뉴를 리스트로 표시
        self.menu_list_widget = QListWidget(self)
        self.menu_list_widget.addItem("짜장면")
        self.menu_list_widget.addItem("짬뽕")
        self.menu_list_widget.addItem("탕수육")

        layout.addWidget(QLabel("추천 메뉴:"))
        layout.addWidget(self.menu_list_widget)

        self.setLayout(layout)

class MainWindow(QDialog):  # QDialog로 수정
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Main Window')

        # button.ui 파일 로드 (Qt Designer로 만든 파일)
        loadUi('button.ui', self)

        # 각 버튼에 대한 클릭 이벤트 연결
        self.pushButton.clicked.connect(self.show_popup1)
        self.pushButton_2.clicked.connect(self.show_popup2)
        self.pushButton_3.clicked.connect(self.show_popup3)

    def show_popup1(self):
        # 첫 번째 팝업 창 띄우기
        self.popup1 = PopupDialog1()
        self.popup1.exec_()

    def show_popup2(self):
        # 두 번째 팝업 창 띄우기
        self.popup2 = PopupDialog2()
        self.popup2.exec_()

    def show_popup3(self):
        # 세 번째 팝업 창 띄우기
        self.popup3 = PopupDialog3()
        self.popup3.exec_()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
