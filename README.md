
# sim_mpc
# Autonomous Navigation System

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

### 2. acados solver 실행
```bash
cd src/mpc_controller/mpc_controller/
python3 acados_mpc_controller.py  
```
rviz 상에서 초기 위치와 goal pose를 찍어주면 a*로 경로를 생성하고 이를 추종할 수 있는 cmd_vel을 publish 합니다. 


## 주요 토픽

- `/scan`: LiDAR 데이터
- `/costmap`: 생성된 Cost map
- `/planned_path`: A*로 계획된 경로
- `/cmd_vel`: 로봇 제어 명령
- `/goal_pose`: 목표 위치


