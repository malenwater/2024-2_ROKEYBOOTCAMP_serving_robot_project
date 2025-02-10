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


# 프로젝트 문제점과 해결할 방안

|이름|문제점|개선할 방안|역할|
|---|---|---|---|
|[이선우](https://github.com/malenwater)|1. 쓰레드 통신을 할 경우, 일반적인 함수를 넘기거나 하는 방법이 불가능하기 때문에, PyQT의 내부 통신방법을 이용해서 통신해야한다.  <br/> 2. 처음 기획까지는 좋았으나 점차 팀원의 능력을 고려하지 않은 지시와 소프트웨어 아키텍쳐가 있지만, 이를 사용자 중심 아키텍쳐와 혼용하여 팀원의 이해도가 떨어졌다. 차후에 남은 기한이 3일이라 설계할 시간이 없어서 팀원에게 일임하는 방식을 일부 팀원에게 할당하고 일부 팀원은 같이하는 형태로 하여 팀원간의 일 할당이 불분명하게 되었다. 이로 인해 최대한의 효율을 뽑아내지 못했다는 것이 문제점이었다. <br/> 3. 코드를 짤 때 코딩 컨벤션을 고려하지 않았었다. 처음이기도 했고, 각자 어느정도 이해하고 있을 거라고 생각했지만, 진행 중에 확인했을 때, 대체 어떤 코드가 무엇인지 이름을 보고 명확하게 무엇을 하는 것인지 보이지 않았다. <br/> 4. 이미지와 mp3를 깃허브에 올렸다.|1. 쓰레드 통신의 경우, 큐를 만들어서 값을 공유하거나, PyQT에서 제공하는 signal을 통해 해결할 수 있다는 것을 다른 팀원을 통해 알 수 있었다. <br/> 2와 3. 팀원간 원활한 소통이 안되었던 이유는 명확한 설계가 없었기 때문에 팀원들이 동기화된 설계를 이해할 수 없었던 것 같다. 또한 동일하게 코딩 컨벤션 또한 하나의 파일에 하나의 클래스를 두고, import 하는 형식을 지키자고 말을 하고 써 놓았다면 유지보수가 쉬운 코드가 나왔을 것이다. 이는 프로젝트 초기에 정해야하는 설계와 코딩 컨벤션의 부재로 인한 어려움이 프로젝트 진행시의 큰 어려움이었고, 차후에 할 프로젝트에서는 명확한 설계와 코딩 컨벤션을 설립하고 진행할려고 한다. <br/> 4. 이미지와 mp3는 개인 저작권이 있기 때문에 넣으면 안된다. 그러므로 AI를 통해 만든 이미지로 대체하고, 또한 mp3 또한 자체적으로 만들어 넣는다.|코드 취합, 서빙로봇 도착 알림 서비스 기능, 키오스크 UI, 데이터베이스 구축|
|[최범석](https://github.com/ausudu)|더미|더미|일일 메뉴, 일일 매출, 메뉴별 매출 기능 서비스, 키오스크 UI와 주방 UI의 Node 간 통신 구축|
|[김영수](https://github.com/youngsoo-kim-123)|1. 큐티로 인터페이스를 만들 때, 큐티의 이벤트 루프랑 rclpy가 서로 충돌해서 결과적으로 rclpy 이벤트 루프 실행 중 큐티 코드가 돌아가지 않음. <br/> 2. 다른 사람이 작성한 코드를 볼 때, 이 코드가 무엇인지 확인 하는게 힘들었음,<br/> 3. 데이터 타입, 통신이름이 원활히 공유되야 하는데, 부족함이 있었음. |1. 앞으로는 ros2코드를 짜기 전에 서로 충돌할 이벤트 루프들을 고려해 스레드를 먼저 설계함으로 이러한 문제점이 다시 일어나지 않게 할것. <br/> 2.  다음 부턴 주석 처리를 더 신경쓸것 <br/> 3. 체계적으로 데이터 타입과 통신이름을 관리할 방법 강구|주방 UI, 서빙 로봇 모드 기능 구축|
|[한건희](https://github.com/ghgue)|코드에 대한 이해도 부족으로 작업 시간을 단축시키는 데에 실패를 하였다. 주어진 상황에서 시간을 최대한 효율적으로 사용하기 위해 UI제작에만 가장 많은 시간을 소요했던 것이 아쉽다. 다만, 이로 인해 PyQT를 활용한 UI제작에 대한 숙련도를 향상시킬 수 있었고, 처음 경험해보는 프로젝트에서, 코딩한 기능이 직접 구현되는 것을 볼 수 있었기 때문에 주행 로봇에 대한 전반적인 이해도를 높일 수 있었다. 첫 프로젝트에 참여하면서 얻은 지식을 기반으로 향후 프로젝트의 작업 시간 단축에 이득을 볼 것이라는 기대를 한다. 또한 여러 명이서 짠 코드를 취합해보니, 시간 부족으로 각자 짠 코드가 어떠한 지에 대한 해석을 하지 못했다.|프로젝트에 쓰인 모든 코드를 다른 사람이 보아도 이해할 수 있도록 주석 처리에 신경을 더 써야 한다.|키오스크 UI, 서빙 로봇 도착 소리 서비스 기능|
