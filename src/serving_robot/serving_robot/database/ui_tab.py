import sys
import csv
from PyQt5.QtWidgets import QApplication, QDialog, QPushButton, QVBoxLayout, QLabel, QListWidget
from PyQt5.uic import loadUi
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from datetime import datetime, timedelta
from PyQt5.QtGui import QFont

plt.rcParams['axes.unicode_minus'] =False

plt.rcParams['font.family'] = 'NanumGothic'




class PopupDialog1(QDialog):
    def __init__(self, data_file='sales_data.csv'):
        super().__init__()
        self.setWindowTitle('일일 매출')
        self.setGeometry(100, 100, 800, 600)

       
        layout = QVBoxLayout()
        self.canvas = FigureCanvas(plt.figure(figsize=(8, 6)))
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        self.dates, self.sales = self.load_sales_data(data_file)
        self.plot_sales_data()

    def load_sales_data(self, file_path):
        dates = []
        sales = []
        today = datetime.now().strftime('%Y/%m/%d')
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                reader = csv.DictReader(file)
                for row in reader:
                    dates.append(row['date'])
                    sales.append(int(row['sales']))
        except (FileNotFoundError, KeyError, ValueError):
            dates = [(datetime.now() - timedelta(days=i)).strftime('%Y/%m/%d') for i in range(3, -1, -1)]
            sales = [10000, 20000, 15000, 100000]
        return dates, sales

    def plot_sales_data(self):
        ax = self.canvas.figure.add_subplot(111)
        ax.clear()
        ax.plot(self.dates, self.sales, marker='o', linestyle='-', color='b')
        ax.set_title("일일 매출")
        ax.set_xlabel("날짜")
        ax.set_ylabel("매출")
        max_sales = max(self.sales) 
        ax.set_ylim(0, max_sales + max_sales * 0.1)
        ax.grid(True)
        self.canvas.draw()


class PopupDialog2(QDialog):
    def __init__(self, menu_items=None, sales=None):
        super().__init__()
        self.setWindowTitle('메뉴별 매출')
        self.setGeometry(100, 100, 800, 600)


        layout = QVBoxLayout()
        self.canvas = FigureCanvas(plt.figure(figsize=(8, 6)))
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        self.menu_items = menu_items if menu_items else ['짜장면', '짬뽕', '탕수육', '김치볶음밥']
        self.sales = sales if sales else [20, 15, 25, 10]

        self.plot_menu_sales_data()

    def plot_menu_sales_data(self):
        ax = self.canvas.figure.add_subplot(111)
        ax.clear()
        ax.bar(self.menu_items, self.sales, color=['orange', 'green', 'blue', 'red'])
        ax.set_title("메뉴별 매출")
        ax.set_xlabel("메뉴")
        ax.set_ylabel("매출")
        ax.grid(True, axis='y')
        ax.set_ylim(0, max(self.sales) + 10)
        self.canvas.draw()


class PopupDialog3(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('선호 메뉴')
        self.setGeometry(100, 100, 400, 300)

        layout = QVBoxLayout()
        
        # 메뉴와 개수를 표시할 리스트
        menu_items = ['짜장면', '짬뽕', '탕수육']
        sales_count = [3, 2, 2]  # 각 메뉴의 개수 (예시)

        # QListWidget 생성
        self.menu_list_widget = QListWidget(self)
        
        # 메뉴와 개수를 리스트 항목에 추가
        for item, count in zip(menu_items, sales_count):
            self.menu_list_widget.addItem(f"{item} - {count}개")  # 메뉴명과 개수 표시

        layout.addWidget(QLabel("선호 메뉴:"))
        layout.addWidget(self.menu_list_widget)
        self.setLayout(layout)



class MainWindow(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Main Window')
        loadUi('button.ui', self)

        self.pushButton.clicked.connect(self.show_popup1)
        self.pushButton_2.clicked.connect(self.show_popup2)
        self.pushButton_3.clicked.connect(self.show_popup3)

    def show_popup1(self):
        self.popup1 = PopupDialog1()
        self.popup1.exec_()

    def show_popup2(self):
        menu_items = ['짜장면', '짬뽕', '탕수육', '김치볶음밥']
        sales = [22, 18, 30, 25]
        self.popup2 = PopupDialog2(menu_items, sales)
        self.popup2.exec_()

    def show_popup3(self):
        self.popup3 = PopupDialog3()
        self.popup3.exec_()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
