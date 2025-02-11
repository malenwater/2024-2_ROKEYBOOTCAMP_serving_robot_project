# 2024-2_ROKEYBOOTCAMP_serving_robot_project_1
2024년 2기 1주차, ROKEY Boot 캠프의 터틀 봇을 이용한 서빙 로봇 서비스를 만드는 프로젝트입니다.     
이 프로젝트는 터틀봇 시뮬레이션을 서빙 로봇으로 대체하였고, UI는 PyQT로, ROS를 통해 주방 디스플레이와 키오스크, 서빙로봇 간 통신을 하도록 제작하였다.

# 해당 프로젝트 사용법

먼저 사용자의 쉘 경로는 모두 본 프로젝트의 WS에서 작업을 진행합니다.   
ROS 프로젝트를 위해 파이썬 패키지의 의존성을 설치하기 위해서 아래 명령어를 실행한다.

    pip install -r ./src/serving_robot/requirements.txt 

이후 다른 ROS2 프로젝트와 같이 실행한다.

    colcon build
    source install/setup.bash

본 프로젝트에서는 MySQL을 통한 주문을 저장하여 관리한다. MySQL 초기화는 아래 코드를 실행한다. 해당 sql 자동완성에서는 유저를 생성하는 것은 보안상 위험하기 때문에 따로 유저를 생성하여 접근하여야 한다.

    mysql -u root -p orders_db < ./src/serving_robot/resource/database/init.sql
    
본 프로젝트에서 MySQL을 사용하기 위해서는 자체 내부에 MySQL을 위한 환경설정 파일인 .env를 만든다.

    touch ./src/serving_robot/.env

.env 파일 내부에는 아래 내용을 넣으며 알맞은 내용을 넣는다.

    DB_HOST = {MySQL의 서버 주소}
    DB_USER = {유저아이디}
    DB_PASSWORD = {유저아이디 비밀번호}
    DB_NAME = Restaurant

유저 생성이 어려울 경우, root를 통해 접근한다.

    DB_HOST = 127.0.0.1
    DB_USER = root
    DB_PASSWORD = {root 비밀번호}
    DB_NAME = Restaurant

본 프로젝트에서 데이터베이스를 연동한 그래프를 띄울 때 사용하는 폰트를 아래 명령어로 다운 받는다.

    sudo apt-get install fonts-nanum*
    rm -rf ~/.cache/matplotlib/*
    fc-cache -fv

위와 같이 본 프로젝트의 ROS2 패키지를 준비한다. 총 4개의 본 프로젝트 파일과 `터틀봇3 - ROBOTIS e-Manual - 로보티즈`에서 제공하는 NAV의 시뮬레이션 2개의 launch 파일을 기준으로 동작한다.

    ros2 run serving_robot test_server 
    ros2 run serving_robot kiosk_ui 
    ros2 run serving_robot robot_sub 
    ros2 run serving_robot test_client 

위 해당 코드는 각각의 shell에서 위에서부터 순서대로 동작시킨다.     
만약 위에 4개 run을 하기 싫다면 동일한 역할을 하는 launch를 하면된다.

        ros2 launch serving_robot kiosk.launch.py

위 launch파일이 순서에 맞춰서 위 4개 run 파일을 실행해준다.
그런 다음 두 개의 shell을 띄어서 아래 명령어를 실행한다.

    export TURTLEBOT3_MODEL=waffle

해당 명령어를 두 shell에 실행한 후 각각의 쉘에서 아래 명령어를 실행합니다.

    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=./src/serving_robot/resource/map/map.yaml

이후, 키오스크 UI와 주방 디스플레이을 조정하여 고객, 주방장, 로봇의 상호작용을 볼 수 있다.
키오스크 번호는 9번까지 있으며 해당 번호는 serving_robot 패키지 내 코드 패키지 kiosk의 kiosk_ui.py 파일의 KioskDialog 클래스 내 self.table_number 번호를 1 ~ 9번으로 변경하여 주문 받는 키오스크를 조정할 수 있다.

# 프로젝트 설계도 및 간단한 설명
<div align="center">
  <img src="https://github.com/user-attachments/assets/ded49433-739e-4807-bc6d-d83a65efbb55" alt="사용자 중심의 아키텍쳐"/>
  <br><b><이미지 1, 사용자 중심의 아키텍쳐></b><br><br>

  <img src="https://github.com/user-attachments/assets/0807cbae-07b3-4075-b294-b2848d3abbaa" alt="기능별 아키텍쳐"/>
  <br><b><이미지 2, 사용자 중심의 기능별 아키텍쳐></b><br><br>

  <img src="https://github.com/user-attachments/assets/0181d145-527b-489d-9800-a91a6dda6427" alt="시스템 아키텍쳐"/>
  <br><b><이미지 3, 시스템 아키텍쳐></b><br><br>

  <img src="https://github.com/user-attachments/assets/5daffed0-034b-4967-9e0d-3bfd4e94fa4d" alt="소프트웨어 아키텍쳐"/>
  <br><b><이미지 4, 소프트웨어 아키텍쳐></b>
</div>

- 이미지 1 : 사용자 중심으로 어떤 기능을 주는 지에 대한 설계도
- 이미지 2 : 사용자 중심으로 기능별 어떤 기능을 가지는지에 대한 설계도
- 이미지 3 : 전체적인 시스템 설계도
- 이미지 4 : 소프트웨어와 기능 간의 전체적인 설계도  



