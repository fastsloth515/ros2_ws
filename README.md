Launch Commands Summary

1. RealSense 카메라 노드

RealSense D435/D455 카메라의 RGB·Depth 스트림을 출력하고 Depth–Color 정렬을 수행하는 노드

실행 명령:

ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640x480x30 \
  rgb_camera.color_profile:=640x480x30 \
  enable_depth:=true \
  enable_color:=true \
  pointcloud.enable:=false \
  align_depth.enable:=true

2. Segmentation 노드 (jetson_quantization-master)

TensorRT로 최적화된 세그멘테이션 모델을 실행해 실시간 클래스 마스크를 생성하는 패키지

실행 명령:

cd ~/ros2_ws/src/jetson_quantization-master
python3 demo_trt.py

3. GPU PointCloud 생성 노드 (pointcloud_xyzrgb)

Depth 이미지와 세그멘테이션 결과를 기반으로 CUDA를 사용해 XYZRGB 포인트클라우드를 생성하는 패키지

실행 명령:

ros2 run pointcloud_xyzrgb pointcloud_gpu_node

4. BEV Occupancy Grid 노드 (bev_cuda)

포인트클라우드를 입력받아 CUDA 기반 Bird’s-Eye-View(Top-view) Occupancy Grid를 생성하는 패키지

실행 명령:

ros2 launch bev_cuda bev.launch.py

5. GPS 기반 Global Planner (gps_nav)

RTK-GPS 위치와 로봇 오도메트리를 기반으로 전역 목표 위치를 계산하고 /dxdy를 퍼블리시하는 글로벌 플래너 패키지

실행 명령:

cd ~/ros2_ws/src/gps_nav/gps_nav
ros2 run gps_nav planner_server

6. DWA Local Planner (dwa_nav)

BEV Occupancy Grid를 기반으로 장애물 회피 경로를 계산해 속도 명령(/cmd)을 생성하는 로컬 플래너 패키지

실행 명령:

ros2 run dwa_nav dwa_node

7. Go2 제어 브리지 노드 (unitree_ros2_example)

DWA 또는 GPS 플래너의 /cmd 명령을 Unitree Go2 제어 명령 형식으로 변환해 로봇에 적용하는 브리지 패키지

실행 명령:

ros2 run unitree_ros2_example dwa2go2_node

