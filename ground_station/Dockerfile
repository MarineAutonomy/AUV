# Ground-station ROS 2 Humble image: acoustic modem (role a) for MAV-GUI AppImage PCs.
# Build+push (repo root):  ./build_gs_docker.sh 1.1
# Local build:  docker build --build-arg GS_IMAGE_VERSION=1.1 -t aatmaj9/auv-ground-station:1.1 -f Dockerfile .
#
# Host needs: Docker, USB modem under /dev, cyclonedds.xml, host network.
# Xbox teleop is NOT here — MAV-GUI uses the browser Gamepad API → vehicle rosbridge.

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
      python3-colcon-common-extensions \
      python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir wlmodem pyserial crcmod cobs

WORKDIR /ws
COPY ros2_ws/src/modem_m64 /ws/src/modem_m64

RUN bash -lc "source /opt/ros/humble/setup.bash && \
      colcon build --packages-select modem_m64 --symlink-install"

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

VOLUME ["/gs", "/tmp"]

ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
