임형태 박사님의 [카이스트 수업자료](https://github.com/LimHyungTae/mcl_2d_lidar_ros) 를 따라서 직접 구현하고자 만듦.  
2D LiDAR 대신 두 개의 리얼센스 스테레오 카메라-RS 435, RS 455 를 사용.  
deque를 사용하는 전체적인 형태는 동일  
-> 2차원 거리 배열을 3차원 점군으로 확장.  
-> 맵에 점유 정도를 표기하기 위한 식 수정.