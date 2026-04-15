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
    # Dipendenze MoveIt e core
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
    # Franka (pacchetti binari stabili)
    ros-humble-franka-msgs \
    ros-humble-franka-gripper \
    ros-humble-realtime-tools \
    ros-humble-control-msgs \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-visualization-msgs \
    ros-humble-action-msgs \
    ros-humble-tf2-msgs \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-controller-manager \
    ros-humble-ros2controlcli \
    ros-humble-urdf \
    ros-humble-rviz2 \
    iputils-ping \
    openssh-client \
    && rm -rf /var/lib/apt/lists/*

# 2. Installazione dipendenze Python per VLM e YOLO
RUN pip3 install --no-cache-dir \
    google-genai \
    pydantic \
    pillow \
    opencv-python \
    "numpy<2" \
    ultralytics \
    scipy

# 3. Setup Workspace e Franka da sorgenti (per Humble)
WORKDIR /mm_ws

# Clonazione franka_ros2 e rimozione immediata dei duplicati conflittuali
RUN mkdir -p /mm_ws/src && cd /mm_ws/src && \
    git clone https://github.com/frankaemika/franka_ros2.git -b humble && \
    # Risoluzione errore "Multiple packages found with the same name"
    rm -rf franka_ros2/franka_gazebo_bringup

# Copia i tuoi pacchetti custom
COPY src/ /mm_ws/src/
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Installazione dipendenze tramite rosdep
RUN apt-get update && \
    . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y -r && \
    rm -rf /var/lib/apt/lists/*

# Build globale del workspace (Franka + Custom)
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 4. Configurazione Bash & Alias
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'if [ -f /mm_ws/install/setup.bash ]; then source /mm_ws/install/setup.bash; fi' >> /root/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /root/.bashrc && \
    echo 'alias cbp="colcon build --symlink-install --packages-select"' >> /root/.bashrc && \
    echo 'alias ros_source="source /opt/ros/humble/setup.bash && source install/setup.bash"' >> /root/.bashrc

# 5. RealSense drivers
RUN apt-get update && apt-get install -y \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
