# Usa l'immagine ufficiale ROS 2 Humble Desktop come base
FROM osrf/ros:humble-desktop

# Disabilita le interazioni durante l'installazione
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# 1. Installazione strumenti di sistema e dipendenze ROS 2 STABILI
RUN apt-get update && apt-get install -y \
    git \
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
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Inizializza rosdep (gestore dipendenze ROS)
RUN rosdep init || true

# 2. Installazione dipendenze Python per VLM e YOLO (Piazzato in ALTO per ottimizzare la CACHE)
RUN pip3 install --no-cache-dir \
    google-genai \
    pydantic \
    pillow \
    opencv-python \
    "numpy<2" \
    ultralytics \
    scipy

# Copiamo I file sorgente per far capire a rosdep cosa manca
WORKDIR /mm_ws
COPY src /mm_ws/src

RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble \
    && rm -rf /mm_ws/src/* \
    && rm -rf /var/lib/apt/lists/*


# 3. Setup Workspace
WORKDIR /mm_ws

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Ensure ROS setup is available in interactive shells (e.g. docker exec bash)
COPY local_bashrc /root/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo 'if [ -f /mm_ws/install/setup.bash ]; then source /mm_ws/install/setup.bash; fi' >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc

# 4. RealSense drivers — layer separato per sfruttare la cache Docker
# Per aggiornare solo questa parte: docker compose build (2 min invece di 20)
RUN apt-get update && apt-get install -y \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]