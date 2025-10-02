#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/humble/setup.bash
source /home/nvidia/KIST/VLM_AutonomousDriving/unitree_ros2/cyclonedds_ws/install/setup.sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eno1" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
