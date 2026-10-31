# Builds the ROS 2 workspace (every package under src/) on ros:jazzy-ros-base.
# Build from the repository root:  docker build -f deploy/Dockerfile -t rsi-proj-ros .
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions python3-pytest \
      ros-jazzy-rmw-fastrtps-cpp \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src/ src/

RUN apt-get update \
 && rosdep update --rosdistro jazzy \
 && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
 && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/jazzy/setup.bash \
 && colcon build --event-handlers console_cohesion+ \
 && rm -rf build log

COPY deploy/fastdds/ /ws/deploy/fastdds/
COPY deploy/entrypoint.sh /ws/deploy/entrypoint.sh
RUN chmod +x /ws/deploy/entrypoint.sh

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENTRYPOINT ["/ws/deploy/entrypoint.sh"]
CMD ["bash"]
