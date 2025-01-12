from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QLabel, QListWidget, QTableWidget, QTableWidgetItem
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import sys

class SalesDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("매출 현황판")
        self.setGeometry(100, 100, 800, 600)
        self.initUI()
    
    def initUI(self):
        main_widget = QWidget()
        main_layout = QVBoxLayout()
        
        # 상단 레이아웃 (일일 매출 그래프 & 메뉴별 매출 차트)
        top_layout = QHBoxLayout()
        
        # 일일 매출 그래프 (Matplotlib 그래프)
        daily_sales_canvas = self.create_bar_chart("일일 매출", [100, 200, 150, 300, 250])
        top_layout.addWidget(daily_sales_canvas)
        
        # 메뉴별 매출 차트 (Matplotlib 그래프)
        menu_sales_canvas = self.create_bar_chart("메뉴별 매출", [50, 120, 90, 180, 140])
        top_layout.addWidget(menu_sales_canvas)
        
        # 하단 레이아웃 (선호 메뉴)
        bottom_layout = QVBoxLayout()
        preference_label = QLabel("선호 메뉴")
        preference_list = QListWidget()
        preference_list.addItems(["메뉴 A", "메뉴 B", "메뉴 C"])
        bottom_layout.addWidget(preference_label)
        bottom_layout.addWidget(preference_list)
        
        # 레이아웃 결합
        main_layout.addLayout(top_layout)
        main_layout.addLayout(bottom_layout)
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

    def create_bar_chart(self, title, data):
        fig, ax = plt.subplots()
        ax.bar(range(len(data)), data)
        ax.set_title(title)
        canvas = FigureCanvas(fig)
        return canvas

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SalesDashboard()
    window.show()
    sys.exit(app.exec_())
