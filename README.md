# parking_hs


1. cap에 lidar stop 후 꺾는 영상 1개
2. cap1에 정지선 영상 1개 넣기

코드 실행하고 
rqt->  plugins → topics → message publisher → liderstop true(체크)로 바꿔주기

# Parking_test
1. contour로 테두리 그리는 것까지 완성
2. 카메라로 촬영한 후 실제 사이즈 if문으로 비교 요망


# Stopline => vision header folder & stopline_2022.cpp
1. vision header 폴더에 있는 파일 2개 다운
2. catkin_ws/src/stopline_2021 에서 cmakelist 수정하기
3. include_directories("경로쓰기") 
4. stopline_2022.cpp가 main

# Parking_0711
1. 코드 실행하려면 우선 vision_publish_example.cpp를 켜야합니다.
2. 그리고 parking_0711.cpp를 켜주세욥!
3. opencv의 tracking 모듈이 꼭 필요한 아이입니다!!!
4. 라이다 true는 rostopic pub /Lidar_Stop std_msgs/Bool true 
