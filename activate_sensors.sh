#!/usr/bin/env bash
echo "🔄 Activating sensors..."

# bash -ic loads ~/.bashrc (ROS already sourced). Do not call sros2/scode again.
# Detached launch — MAV-GUI polls process status for the green lights.
docker exec -d auv bash -ic '
ros2 launch dvl_a50 dvl_a50.launch.py ip_address:=192.168.194.95 &
ros2 run ping360_sonar ping360.py --ros-args -p device:=/dev/ping360 &
ros2 run ping_sonar_ros ping1d_node --ros-args -p port:=/dev/ping2 &
ros2 launch sbg_driver sbg_device_launch.py &
(sleep 2 && ros2 run v4l2_camera v4l2_camera_node --ros-args -r __ns:=/front -p video_device:=/dev/frontcam -p image_size:=[640,480] -p framerate:=20.0 -p pixel_format:=YUYV -p output_encoding:=yuv422_yuy2) &
(sleep 2 && ros2 run v4l2_camera v4l2_camera_node --ros-args -r __ns:=/bottom -p video_device:=/dev/bottomcam -p image_size:=[640,480] -p framerate:=20.0 -p pixel_format:=YUYV -p output_encoding:=yuv422_yuy2) &
ros2 run modem_m64 modem_node --ros-args -r __ns:=/auv -p role:=b -p port:=/dev/modem &
ros2 run arduino_ps arduino_ps &
ros2 launch sidescan_ros2 sidescan.launch.py &
'

echo "🟢 Sensor launch commands issued!"
