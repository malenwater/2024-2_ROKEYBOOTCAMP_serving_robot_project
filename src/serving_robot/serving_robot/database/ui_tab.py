import sys
import csv
import rclpy
from rclpy.node import Node
import mysql.connector
from PyQt5.QtWidgets import QApplication,  QDialog, QPushButton, QVBoxLayout, QLabel, QListWidget
from PyQt5.uic import loadUi
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from datetime import datetime, timedelta
from PyQt5.QtGui import QFont
import os
sys.path.append('./src/serving_robot')
from serving_robot.database import DB_HOST, DB_USER, DB_PASSWORD, DB_NAME

plt.rcParams['axes.unicode_minus'] =False

plt.rcParams['font.family'] = 'NanumGothic'

class MySQLConnector:
    def __init__(self, host, user, password, database):
        self.host = host
        self.user = user
        self.password = password
        self.database = database
        self.conn = None
        self.cursor = None

    def connect(self):
        try:
            self.conn = mysql.connector.connect(
                host=self.host,
                user=self.user,
                password=self.password,
                database=self.database
            )
            self.cursor = self.conn.cursor()
            print("데이터베이스 연결 성공!")
        except mysql.connector.Error as err:
            print(f"데이터베이스 연결 실패: {err}")
            return False
        return True

    def fetch_data(self, query):
        try:
            self.cursor.execute(query)
            results = self.cursor.fetchall()
            return results
        except mysql.connector.Error as err:
            print(f"쿼리 실행 오류: {err}")
            return None

    def close(self):
        if self.cursor:
            self.cursor.close()
        if self.conn:
            self.conn.close()
        print("데이터베이스 연결 종료.")



class PopupDialog1(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('일일 매출')
        self.setGeometry(100, 100, 800, 600)

        layout = QVBoxLayout()
        self.canvas = FigureCanvas(plt.figure(figsize=(8, 6)))
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        # MySQL 데이터베이스 연결
        self.menu_items, self.sales = self.fetch_sales_data()
        self.plot_sales_data()

    def fetch_sales_data(self):
        db_connector = MySQLConnector(
            host=DB_HOST,       # __init__.py에서 가져온 환경 변수
            user=DB_USER,
            password=DB_PASSWORD,
            database=DB_NAME
        )

        dates = []
        sales = []

        if db_connector.connect():
            query = """
            SELECT 
                DATE(orders.created_date) AS order_date,
                SUM(orders_product.quantity * products.price) AS total_sales
                FROM orders
                INNER JOIN orders_product ON orders.order_id = orders_product.order_id
                INNER JOIN products ON orders_product.product_id = products.product_id
                WHERE DATE(orders.created_date) BETWEEN DATE_SUB(CURDATE(), INTERVAL 3 DAY) AND CURDATE()
                GROUP BY DATE(orders.created_date)
                ORDER BY order_date ASC  -- 날짜 오름차순 정렬 추가
            """

            results = db_connector.fetch_data(query)

            if results:
                for row in results:
                    dates.append(row[0].strftime('%Y-%m-%d'))
                    sales.append(float(row[1]))

            db_connector.close()

        return dates, sales

    def plot_sales_data(self):
        ax = self.canvas.figure.add_subplot(111)
        ax.clear()
        ax.plot(self.menu_items, self.sales, marker='o', linestyle='-', color='b')
        ax.set_title("일일 매출")
        ax.set_xlabel("날짜")
        ax.set_ylabel("매출 (₩)")
        ax.grid(True)
        ax.set_ylim(0, max(self.sales) + max(self.sales) * 0.1)
        self.canvas.draw()




class PopupDialog2(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('메뉴별 매출')
        self.setGeometry(100, 100, 800, 600)

        layout = QVBoxLayout()
        self.canvas = FigureCanvas(plt.figure(figsize=(8, 6)))
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        # MySQL에서 데이터 불러오기
        self.menu_items, self.sales = self.fetch_sales_data()

        # 데이터 시각화
        if self.menu_items and self.sales:
            self.plot_menu_sales_data()
        else:
            print("데이터를 불러오는 데 실패했습니다.")

    def fetch_sales_data(self):
        db_connector = MySQLConnector(
            host=DB_HOST,       # __init__.py에서 가져온 환경 변수
            user=DB_USER,
            password=DB_PASSWORD,
            database=DB_NAME
        )

        menu_items = []
        sales = []

        if db_connector.connect():
            query = """
                SELECT 
                    products.name, 
                    SUM(orders_product.quantity * products.price) AS total_price
                FROM orders
                INNER JOIN orders_product 
                    ON orders.order_id = orders_product.order_id
                INNER JOIN products 
                    ON orders_product.product_id = products.product_id
                WHERE DATE(orders.created_date) = CURDATE()
                GROUP BY products.name;
            """

            results = db_connector.fetch_data(query)

        if results:
            for row in results:
                menu_items.append(row[0])  # 메뉴 이름
                sales.append(row[1])       # 총 매출

            db_connector.close()

        if not menu_items:
            print("메뉴 데이터를 찾을 수 없습니다.")
            menu_items = ['데이터 없음']
            sales = [0]

        return menu_items, sales

    def plot_menu_sales_data(self):
        ax = self.canvas.figure.add_subplot(111)
        ax.clear()
        ax.bar(self.menu_items, self.sales, color='skyblue')
        ax.set_title("메뉴별 매출")
        ax.set_xlabel("메뉴")
        ax.set_ylabel("매출 (₩)")
        ax.grid(True, axis='y')
        ax.set_ylim(0, max(self.sales) + 1000)
        self.canvas.draw()




class PopupDialog3(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('선호 메뉴')
        self.setGeometry(100, 100, 400, 300)

        layout = QVBoxLayout()

        # QListWidget 생성
        self.menu_list_widget = QListWidget(self)
        layout.addWidget(QLabel("선호 메뉴:"))
        layout.addWidget(self.menu_list_widget)
        self.setLayout(layout)

        # MySQL 데이터 불러오기
        self.load_preferred_menu()

    def load_preferred_menu(self):
        # MySQL 연결
        db_connector = MySQLConnector(
            host=DB_HOST,       # __init__.py에서 가져온 환경 변수
            user=DB_USER,
            password=DB_PASSWORD,
            database=DB_NAME
        )

        if db_connector.connect():
            # 오늘 날짜 기준 메뉴별 판매량 조회
            query = """
            SELECT 
                products.name, 
                SUM(orders_product.quantity) AS total_quantity
            FROM orders
            INNER JOIN orders_product 
                ON orders.order_id = orders_product.order_id
            INNER JOIN products 
                ON orders_product.product_id = products.product_id
            WHERE DATE(orders.created_date) = CURDATE()
            GROUP BY products.name
            ORDER BY total_quantity DESC;
            """

            results = db_connector.fetch_data(query)

            if results:
                for name, total_quantity in results:
                    self.menu_list_widget.addItem(f"{name} - {total_quantity}개")
            else:
                self.menu_list_widget.addItem("데이터가 없습니다.")

            db_connector.close()
        else:
            self.menu_list_widget.addItem("데이터베이스 연결 실패")



class MainWindow(QDialog):
    def __init__(self,parent=None):
        super().__init__(parent=parent)
        self.setWindowTitle('Main Window')
        loadUi('./src/serving_robot/resource/ui/button.ui', self)

        self.pushButton.clicked.connect(self.show_popup1)
        self.pushButton_2.clicked.connect(self.show_popup2)
        self.pushButton_3.clicked.connect(self.show_popup3)
        
    def show_popup1(self):
        self.popup1 = PopupDialog1()
        self.popup1.exec_()

    def show_popup2(self):
        self.popup2 = PopupDialog2()
        self.popup2.exec_()


    def show_popup3(self):
        self.popup3 = PopupDialog3()
        self.popup3.exec_()


class ServingRobotNode(Node):
    def __init__(self):
        super().__init__('serving_robot_node')

        # QApplication을 ROS 노드의 생성자에서 시작
        self.app = QApplication(sys.argv)
        self.main_window = MainWindow()
        self.main_window.show()

    def run(self):
        # ROS 2 이벤트 루프와 PyQt5 이벤트 루프를 동시에 실행하기 위한 방법
        self.app.exec_()

def main(args=None):
    rclpy.init(args=args)
    
    # ROS 2 노드와 UI 애플리케이션을 함께 실행
    node = ServingRobotNode()
    node.run()
    
    # ROS 2 이벤트 루프
    rclpy.shutdown()

if __name__ == '__main__':
    main()
