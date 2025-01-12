# serving_robot/database/db_connector.py

import os
import mysql.connector
from dotenv import load_dotenv

# .env 파일 로드
env_path = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'resource',
    '.env'
)
load_dotenv(dotenv_path=env_path)

DB_HOST = os.getenv('DB_HOST')
DB_USER = os.getenv('DB_USER')
DB_PASSWORD = os.getenv('DB_PASSWORD')
DB_NAME = os.getenv('DB_NAME')


class MySQLConnector:
    def __init__(self):
        self.conn = None
        self.cursor = None

    def connect(self):
        try:
            self.conn = mysql.connector.connect(
                host=DB_HOST,
                user=DB_USER,
                password=DB_PASSWORD,
                database=DB_NAME
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
