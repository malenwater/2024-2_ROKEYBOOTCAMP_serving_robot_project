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

# 프로젝트 문제점과 해결할 방안

|이름|문제점|개선할 방안|
|---|---|---|
|[이선우](https://github.com/malenwater)|1. 쓰레드 통신을 할 경우, 일반적인 함수를 넘기거나 하는 방법이 불가능하기 때문에, PyQT의 내부 통신방법을 이용해서 통신해야한다.  <br/> 2. 처음 기획까지는 좋았으나 점차 팀원의 능력을 고려하지 않은 지시와 소프트웨어 아키텍쳐가 있지만, 이를 사용자 중심 아키텍쳐와 혼용하여 팀원의 이해도가 떨어졌다. 차후에 남은 기한이 3일이라 설계할 시간이 없어서 팀원에게 일임하는 방식을 일부 팀원에게 할당하고 일부 팀원은 같이하는 형태로 하여 팀원간의 일 할당이 불분명하게 되었다. 이로 인해 최대한의 효율을 뽑아내지 못했다는 것이 문제점이었다. <br/> 3. 코드를 짤 때 코딩 컨벤션을 고려하지 않았었다. 처음이기도 했고, 각자 어느정도 이해하고 있을 거라고 생각했지만, 진행 중에 확인했을 때, 대체 어떤 코드가 무엇인지 이름을 보고 명확하게 무엇을 하는 것인지 보이지 않았다. <br/> 4. 이미지와 mp3를 깃허브에 올렸다.|1. 쓰레드 통신의 경우, 큐를 만들어서 값을 공유하거나, PyQT에서 제공하는 signal을 통해 해결할 수 있다는 것을 다른 팀원을 통해 알 수 있었다. <br/> 2와 3. 팀원간 원활한 소통이 안되었던 이유는 명확한 설계가 없었기 때문에 팀원들이 동기화된 설계를 이해할 수 없었던 것 같다. 또한 동일하게 코딩 컨벤션 또한 하나의 파일에 하나의 클래스를 두고, import 하는 형식을 지키자고 말을 하고 써 놓았다면 유지보수가 쉬운 코드가 나왔을 것이다. 이는 프로젝트 초기에 정해야하는 설계와 코딩 컨벤션의 부재로 인한 어려움이 프로젝트 진행시의 큰 어려움이었고, 차후에 할 프로젝트에서는 명확한 설계와 코딩 컨벤션을 설립하고 진행할려고 한다. <br/> 4. 이미지와 mp3는 개인 저작권이 있기 때문에 넣으면 안된다. 그러므로 AI를 통해 만든 이미지로 대체하고, 또한 mp3 또한 자체적으로 만들어 넣는다.|
|[최범석](https://github.com/ausudu)|더미|더미|
|[김영수](https://github.com/youngsoo-kim-123)|더미|더미|
|[한건희](https://github.com/ghgue)|더미|더미|
