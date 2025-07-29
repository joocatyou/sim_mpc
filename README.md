<<<<<<< HEAD
# sim_mpc
=======
# Autonomous Navigation System

Nav2 없이 직접 구현한 자율 주행 시스템입니다.

## 시스템 구성

1. **robot_description**: 로봇 URDF 및 Gazebo 시뮬레이션
2. **obstacle_detection**: LiDAR 센서 기반 장애물 감지 및 Cost map 생성
3. **path_planner**: A* 알고리즘 기반 경로 계획
4. **mpc_controller**: MPC 알고리즘 기반 경로 추종 제어
5. **autonomous_launcher**: 통합 런치 파일

## 빌드 방법

```bash
cd ~/autonomous_navigation_ws
colcon build
source install/setup.bash
```

## 실행 방법

### 1. 전체 시스템 실행
```bash
ros2 launch autonomous_launcher autonomous_navigation.launch.py
```

### 2. 시뮬레이션만 실행
```bash
ros2 launch autonomous_launcher simulation_only.launch.py
```

### 3. 개별 노드 실행
```bash
# Gazebo 시뮬레이션
ros2 launch robot_description gazebo.launch.py

# 장애물 감지
ros2 run obstacle_detection obstacle_detector

# 경로 계획
ros2 run path_planner astar_planner

# MPC 컨트롤러
ros2 run mpc_controller mpc_controller
```

## 사용 방법

1. Gazebo에서 장애물 추가
2. RViz에서 "2D Nav Goal" 툴로 목표점 설정
3. 로봇이 자동으로 경로를 계획하고 추종

## 주요 토픽

- `/scan`: LiDAR 데이터
- `/costmap`: 생성된 Cost map
- `/planned_path`: A*로 계획된 경로
- `/path_markers`: 경로 시각화 마커
- `/cmd_vel`: 로봇 제어 명령
- `/goal_pose`: 목표 위치

## 시각화

RViz에서 다음 항목들을 확인할 수 있습니다:
- 로봇 모델
- LiDAR 스캔 데이터
- Cost map
- 계획된 경로
- 경로 마커
>>>>>>> e02faac (mpc)
