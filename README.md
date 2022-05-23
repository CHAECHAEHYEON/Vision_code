# parking_hs


1. cap에 lidar stop 후 꺾는 영상 1개
2. cap1에 정지선 영상 1개 넣기

코드 실행하고 
rqt->  plugins → topics → message publisher → liderstop true(체크)로 바꿔주기

# Parking_test
1. contour로 테두리 그리는 것까지 완성
2. 카메라로 촬영한 후 실제 사이즈 if문으로 비교 요망


# Stopline => vision header folder & test header.cpp
1. catkin_ws/src/stopline_2021 에서 cmakelist 수정하기
2. include_directories("경로쓰기") 
