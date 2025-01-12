import os
from dotenv import load_dotenv

# 올바른 .env 파일 경로 설정
env_path = './src/serving_robot/resource/.env'

print(f".env 경로 확인: {env_path}")

# .env 파일 로드
if load_dotenv(dotenv_path=env_path):
    print(".env 파일이 성공적으로 로드되었습니다.")
else:
    print(".env 파일 로드 실패")

# 환경변수 출력
DB_HOST = os.getenv('DB_HOST')
DB_USER = os.getenv('DB_USER')
DB_PASSWORD = os.getenv('DB_PASSWORD')
DB_NAME = os.getenv('DB_NAME')

print(f"DB 연결 정보: {DB_HOST}, {DB_USER}, {DB_NAME}")
