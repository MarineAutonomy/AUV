# Ground-station ROS 2 Humble image: joy_node + auv_joy_teleop.
# Build+push (repo root):  ./build_gs_docker.sh 1.0
# Local build:  docker build --build-arg GS_IMAGE_VERSION=1.0 -t aatmaj9/auv-ground-station:1.0 -f Dockerfile .
# Run:    ./scripts/run_joy_docker.sh
#
# Host needs: Docker, joystick device, cyclonedds.xml (mounted), network host for Jetson DDS.

FROM ros:humble-ros-base

ARG GS_IMAGE_VERSION=0.0.0
LABEL org.opencontainers.image.version="${GS_IMAGE_VERSION}" \
      org.opencontainers.image.title="auv-ground-station"

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///gs/cyclonedds.xml \
    GS_IMAGE_VERSION="${GS_IMAGE_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-joy \
      python3-colcon-common-extensions \
      python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Package source (built into the image)
WORKDIR /ws
COPY ros2_ws/src/auv_joy_teleop /ws/src/auv_joy_teleop

RUN bash -lc "source /opt/ros/humble/setup.bash && \
      colcon build --packages-select auv_joy_teleop --symlink-install"

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Host mounts: /gs/cyclonedds.xml, /tmp (GUI status/PWM files), /dev/input
VOLUME ["/gs", "/tmp"]

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "auv_joy_teleop", "joy_teleop.launch.py"]
