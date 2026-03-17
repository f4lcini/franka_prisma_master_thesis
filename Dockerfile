# Usa l'immagine ufficiale ROS 2 Humble Desktop come base
FROM osrf/ros:humble-desktop

# Disabilita le interazioni durante l'installazione
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# 1. Installazione strumenti di sistema e dipendenze ROS 2 STABILI
RUN apt-get update && apt-get install -y \
    git \
    nano \
    tree \
    net-tools \
    build-essential \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    # Dipendenze core per compilare sorgenti Franka/IDRA
    ros-humble-moveit-task-constructor-core \
    ros-humble-moveit-ros-perception \
    ros-humble-moveit-visual-tools \
    ros-humble-moveit-servo \
    ros-humble-vision-msgs \
    ros-humble-cv-bridge \
    ros-humble-xacro \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-trajectory-controller \
    ros-humble-position-controllers \
    ros-humble-velocity-controllers \
    ros-humble-ros-ign-gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-ign-ros2-control \
    ros-humble-ros-gz-bridge \
    ros-humble-rosidl-default-generators \
    ros-humble-ament-cmake-clang-format \
    ros-humble-libfranka \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-py-trees-ros-viewer \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-eigen \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-core \
    ros-humble-moveit-msgs \
    ros-humble-eigen3-cmake-module \
    ros-humble-py-trees \
    ros-humble-py-trees-ros \
    ros-humble-shape-msgs \
    ros-humble-franka-msgs \
    ros-humble-franka-hardware \
    ros-humble-franka-gripper \
    ros-humble-franka-bringup \
    ros-humble-franka-example-controllers \
    ros-humble-franka-semantic-components \
    ros-humble-realtime-tools \
    ros-humble-control-msgs \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-visualization-msgs \
    ros-humble-action-msgs \
    ros-humble-tf2-msgs \
    ros-humble-pinocchio \
    ros-humble-generate-parameter-library \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-controller-manager \
    ros-humble-ros2controlcli \
    ros-humble-urdf \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-rviz-marker-tools \
    && rm -rf /var/lib/apt/lists/*

# 2. Installazione dipendenze Python per VLM e YOLO (Piazzato in ALTO per ottimizzare la CACHE)
RUN pip3 install --no-cache-dir \
    google-genai \
    pydantic \
    pillow \
    opencv-python \
    "numpy<2" \
    ultralytics \
    scipy

# 3. Setup Workspace
WORKDIR /mm_ws

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Automate .bashrc configuration with aliases and ROS sourcing
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'if [ -f /mm_ws/install/setup.bash ]; then source /mm_ws/install/setup.bash; fi' >> /root/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /root/.bashrc && \
    echo 'export LC_NUMERIC=en_US.UTF-8' >> /root/.bashrc && \
    echo 'export QTWEBENGINE_DISABLE_SANDBOX=1' >> /root/.bashrc && \
    echo 'export XDG_RUNTIME_DIR=/tmp/runtime-root' >> /root/.bashrc && \
    echo 'alias cbp="colcon build --symlink-install --packages-select"' >> /root/.bashrc && \
    echo 'cbsp() { local PKG_NAME=$1; if [ -z "$PKG_NAME" ]; then echo "Usage: cbsp <package_name>"; return 1; fi; if colcon build --symlink-install --packages-select "$PKG_NAME"; then source install/setup.bash; echo -e "\033[1;32m$PKG_NAME built and sourced.\033[0m"; fi; }' >> /root/.bashrc && \
    echo 'alias ll="ls -alF"' >> /root/.bashrc && \
    echo 'alias la="ls -A"' >> /root/.bashrc && \
    echo 'alias l="ls -CF"' >> /root/.bashrc && \
    echo 'alias ros_source="source /opt/ros/humble/setup.bash && source install/setup.bash"' >> /root/.bashrc

# 4. RealSense drivers — layer separato per sfruttare la cache Docker
# Per aggiornare solo questa parte: docker compose build (2 min invece di 20)
RUN apt-get update && apt-get install -y \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]