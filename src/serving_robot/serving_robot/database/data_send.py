from dotenv import load_dotenv  # 추가
import mysql.connector
import datetime
import sys
import os

# .env 파일 경로 추가 및 로드
env_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../resource/.env'))
load_dotenv(env_path)

# .env에서 DB 정보 불러오기
DB_HOST = os.getenv('DB_HOST')
DB_USER = os.getenv('DB_USER')
DB_PASSWORD = os.getenv('DB_PASSWORD')
DB_NAME = os.getenv('DB_NAME')

class MySQLConnector:
    def __init__(self, host, user, password, database):
        self.host = host
        self.user = user
        self.password = password
        self.database = database
        self.conn = None
        self.cursor = None

    def connect(self):
        print("데이터베이스 연결 시도 중...")  # 디버깅용 출력
        try:
            self.conn = mysql.connector.connect(
                host=self.host,
                user=self.user,
                password=self.password,
                database=self.database
            )
            self.cursor = self.conn.cursor()
            print("데이터베이스 연결 성공!")  # 연결 성공 시 출력
        except mysql.connector.Error as err:
            print(f"데이터베이스 연결 실패: {err}")
            return False
        return True

    def close(self):
        if self.cursor:
            self.cursor.close()
        if self.conn:
            self.conn.close()
        print("데이터베이스 연결 종료.")

    def insert_data(self, query, values):
        """공통 INSERT 실행 메서드"""
        try:
            self.cursor.execute(query, values)
            self.conn.commit()  # 데이터베이스에 변경 사항 커밋
            print("데이터 삽입 성공!")
        except mysql.connector.Error as err:
            print(f"데이터 삽입 오류: {err}")
            return False
        return True

    def insert_order(self, total_price, table_number, created_date, updated_date):
        """orders 테이블에 행 삽입"""
        query = """
            INSERT INTO orders (total_price, table_number, created_date, updated_date)
            VALUES (%s, %s, %s, %s);
        """
        values = (total_price, table_number, created_date, updated_date)
        return self.insert_data(query, values)

    def insert_order_product(self, order_id, product_id, quantity, price, delivery_completed):
        """orders_product 테이블에 행 삽입"""
        query = """
            INSERT INTO orders_product (order_id, product_id, quantity, price, delivery_completed)
            VALUES (%s, %s, %s, %s, %s);
        """
        values = (order_id, product_id, quantity, price, delivery_completed)
        return self.insert_data(query, values)

    def get_next_order_id(self):
        """
        orders 테이블에서 최대 order_id를 찾고 +1.
        수동으로 order_id를 관리하는 경우에만 사용.
        """
        query = "SELECT MAX(order_id) FROM orders"
        self.cursor.execute(query)
        result = self.cursor.fetchone()
        return result[0] + 1 if result[0] else 1
    def get_product_price(self, product_id):
        query = "SELECT price FROM products WHERE product_id = %s"
        try:
            self.cursor.execute(query, (product_id,))
            result = self.cursor.fetchone()
            if result:
                return result[0]  # 가격 반환
            else:
                print(f"상품 ID {product_id}에 대한 가격을 찾을 수 없습니다.")
                return 0  # 기본값 0 반환
        except mysql.connector.Error as err:
            print(f"쿼리 실행 오류 (get_product_price): {err}")
            return 0
        
    def fetch_data(self, query):
        """SELECT 쿼리 실행 후 결과 반환"""
        try:
            self.cursor.execute(query)
            results = self.cursor.fetchall()
            return results
        except mysql.connector.Error as err:
            print(f"쿼리 실행 오류: {err}")
            return None


class DataSender:
    def __init__(self):
        self.db_connector = MySQLConnector(DB_HOST, DB_USER, DB_PASSWORD, DB_NAME)
        if not self.db_connector.connect():
            raise Exception("DB 연결 실패")

    def insert_order(self, order_data):
        """
        order_data 예시:
        {
            "order_id": 10,        # 필요 시
            "total_price": 15000,
            "table_number": 1,
            "created_date": datetime.datetime.now(),
            "updated_date": datetime.datetime.now()
        }
        """
        # 만약 order_id를 직접 넣지 않고, 자동증가를 쓰면
        # query에서는 order_id를 안 넣는 형태가 되어야 합니다.
        query = """
            INSERT INTO orders (total_price, table_number, created_date, updated_date)
            VALUES (%s, %s, %s, %s);
        """
        values = (
            order_data["total_price"],
            order_data["table_number"],
            order_data["created_date"],
            order_data["updated_date"]
        )
        success = self.db_connector.insert_data(query, values)
        if not success:
            print("orders 테이블 삽입 실패")
            return False
        return True

    def insert_order_product(self, order_product_data):
        """
        order_product_data 예시:
        {
            "order_id": 10,
            "product_id": 4,
            "quantity": 2,
            "price": 7000,
            "delivery_completed": 0
        }
        """
        query = """
            INSERT INTO orders_product (order_id, product_id, quantity, price)
            VALUES (%s, %s, %s, %s);
        """
        values = (
            order_product_data["order_id"],
            order_product_data["product_id"],
            order_product_data["quantity"],
            order_product_data["price"],
        )
        success = self.db_connector.insert_data(query, values)
        if not success:
            print("orders_product 테이블 삽입 실패")
            return False
        return True

    def close(self):
        self.db_connector.close()
