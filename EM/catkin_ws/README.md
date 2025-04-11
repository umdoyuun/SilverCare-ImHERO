# 차량형 자율주행 로봇 프로젝트

## 프로젝트 개요

LiDAR 센서 기반의 SLAM과 자율주행이 가능한 차량형 로봇 시스템

### 특징

- IMU/엔코더 없이 LiDAR만으로 자율주행
- Hector SLAM을 이용한 실시간 지도 작성
- ROS Navigation Stack 기반 자율주행
- I2C 통신 기반 모터 제어
- 사람 추종 기능

## 하드웨어 구성

- YDLidar X4
- DC 모터 (PCA9685 I2C 제어)
- 서보 모터 (애커만 조향)
- 카메라 (사람 추종용)

## 사용 플러그인

1. Hybrid A* Planner (hybrid_astar_planner/HybridAStarPlanner)
   - 전역 경로 계획
   - 차량 기구학적 제약 고려
   - Reeds-Shepp 곡선 기반 경로 생성

2. Backoff Recovery (backoff_recovery/BackoffRecovery)
   - 후진 기반 복구 행동
   - 충돌/막힘 상황 대응

## 프로젝트 구성

프로젝트는 다음 ROS 패키지들로 구성되어 있습니다:

1. my_slam
   - Hector SLAM 기반 SLAM 및 Navigation 설정
   - Move Base 파라미터 설정
   - 자율주행을 위한 costmap, local planner 설정
   - 사람 추종 노드 구현

2. hector_slam
   - LiDAR 기반 SLAM
   - scan matching을 통한 로봇 위치 추정
   - 실시간 지도 생성

3. robot_motor_controller
   - I2C 기반 DC/서보 모터 제어
   - cmd_vel 토픽을 모터 제어 신호로 변환

4. ydlidar_ros_driver
   - YDLidar X4 드라이버
   - laser scan 토픽 발행

5. yolo_pkg
   - 딥러닝 기반 사람 인식
   - YOLO 객체 검출
   - 실시간 영상 처리

## 실행 방법

전체 시스템은 단일 launch 파일로 실행됩니다:

```bash
roslaunch my_slam navigation.launch
```

- Hector SLAM
- YDLidar 드라이버
- Motor Controller
- Move Base
- YOLO 객체 검출
- 사람 추종 노드
- RViz

### 주행 모드

시스템은 세 가지 주행 모드를 지원합니다:

1. 정지 모드 (Stop)
   - 모든 동작 정지

2. 추적 모드 (Tracking)
   - YOLO를 통한 실시간 사람 인식
   - 인식된 사람을 자동으로 추적
   - 장애물 회피 기능 포함

3. 자율주행 모드 (Autonomous)
   - RViz에서 2D Nav Goal 지정
   - Hybrid A* 알고리즘 기반 경로 계획
   - 자동 장애물 회피

### 제어 방법

1. 소켓 통신
   - 라즈베리파이와 TCP/IP 통신
   - JSON 형식의 제어 명령 수신

   ```json
   {
       "session_id": "session_123",
       "cameraOn": true,
       "drivingMode": "tracking"
   }
   ```

2. 조이스틱 제어 (Xbox 컨트롤러)
   - 왼쪽 스틱: 전진/후진 (최대 0.6 m/s)
   - 오른쪽 스틱: 좌우 조향 (최대 1.2 rad/s)
   - A 버튼: 주행 모드 변경
   - B 버튼: Recovery 동작 중지
   - X/Y 버튼: 자동/수동 모드 전환

## 주요 설정

- costmap 장애물 인식 범위: 5m
- 후진 기반 recovery behavior
- 차량 크기: 30cm x 16cm
- 최대 선속도: 0.6 m/s
- 최대 회전속도: 1.2 rad/s
- 최소 선속도: 0.1 m/s
- local costmap clearing 활성화
- DWA Local Planner 사용
- YOLO 디스플레이 크기: 960x540

## 참고사항

- IMU/엔코더 없이 LiDAR만으로 위치 추정
- 애커만 조향 방식으로 제자리 회전 불가
- Hector SLAM이 위치 추정 담당
- 사람 추종 모드에서 bbox_center 토픽으로 제어
