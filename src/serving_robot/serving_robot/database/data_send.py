import mysql.connector
import datetime

class OrderManager:
    def __init__(self, db_config):
        self.mydb = mysql.connector.connect(**db_config)
        self.cursor = self.mydb.cursor()

    def create_order(self, order_data, table_number):
        """
        주문 데이터를 받아 orders 및 orders_products 테이블에 저장합니다.

        Args:
            order_data (dict): 주문 데이터 (예: {1: [[2, 1], [1, 1]]}).
            table_number (int): 테이블 번호.

        Returns:
            int: 생성된 order_id, 실패시 None 반환.
        """
        try:
            now = datetime.datetime.now()
            total_price = 0

            # 주문 상품 가격 계산
            for product_id, details in order_data.items():
                for quantity, price in details:
                    total_price += quantity * price

            # orders 테이블에 삽입
            sql = "INSERT INTO orders (total_price, table_number, created_date, updated_date) VALUES (%s, %s, %s, %s)"
            val = (total_price, table_number, now, now)
            self.cursor.execute(sql, val)
            self.mydb.commit()
            order_id = self.cursor.lastrowid # auto increment로 생성된 order_id 가져오기

            # orders_products 테이블에 삽입
            for product_id, details in order_data.items():
                for quantity, price in details:
                    sql = "INSERT INTO orders_product (order_id, product_id, quantity, price, delivery_completed) VALUES (%s, %s, %s, %s, %s)"
                    val = (order_id, product_id, quantity, price, 0) # delivery_completed는 기본값 0 (미완료)
                    self.cursor.execute(sql, val)
            self.mydb.commit()

            return order_id

        except mysql.connector.Error as err:
            print(f"Error: {err}")
            self.mydb.rollback() # 오류 발생 시 롤백
            return None

        finally:
            self.mydb.close()

if __name__ =="__main__":
    # 데이터베이스 연결 설정
    db_config = {
        "host": "192.168.123.8",
        "user": "user2",
        "password": "Rasd123132!",
        "database": "Restaurant"
    }

    # 클래스 인스턴스 생성
    order_manager = OrderManager(db_config)

    # 주문 데이터
    order_data = {1: [[2, 5000], [1, 5000]], 2: [[1,15000]]} # product_id : [[quantity, price], [quantity, price]...]
    table_number = 1

    # 주문 생성
    order_id = order_manager.create_order(order_data, table_number)

    if order_id:
        print(f"주문 생성 완료. order_id: {order_id}")
    else:
        print("주문 생성 실패.")