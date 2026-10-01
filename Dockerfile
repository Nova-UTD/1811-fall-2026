FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
LABEL maintainer="1811"

RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-tf2-eigen \
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-joy \
    python3-serial \
    python3-pygame \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-tf2-tools \
    ros-humble-point-cloud-msg-wrapper \
    build-essential \
    libeigen3-dev \
    libjsoncpp-dev \
    libspdlog-dev \
    libcurl4-openssl-dev \
    libopencv-dev \
    libzip-dev \
    libssl-dev \
    cmake \
    python3-colcon-common-extensions \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir "cmake>=3.24"

RUN git config --global --add safe.directory /vehicle_1811

# Interactive shells land in the workspace, ROS 2 and the built overlay already
# sourced -- no `cd` + two `source` lines at the top of every new terminal.
#
# The base ros:humble entrypoint sources /opt/ros/humble only; it knows nothing
# about this repo's workspace, which is bind-mounted at runtime. So the overlay
# has to be sourced here, and guarded: a fresh clone has no install/ yet, and an
# unguarded source would error on every login.
RUN printf '%s\n' \
    '' \
    '# --- 1811 ---' \
    'source /opt/ros/humble/setup.bash' \
    'if [ -f /vehicle_1811/ros2_ws/install/setup.bash ]; then' \
    '    source /vehicle_1811/ros2_ws/install/setup.bash' \
    'else' \
    '    echo "[1811] workspace not built yet -- run: colcon build --symlink-install"' \
    'fi' \
    'cd /vehicle_1811/ros2_ws' \
    "alias rebuild='colcon build --symlink-install --packages-skip smart_rviz_plugin && source install/setup.bash'" \
    >> /root/.bashrc

WORKDIR /vehicle_1811/ros2_ws