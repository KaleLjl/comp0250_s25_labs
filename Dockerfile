FROM osrf/ros:noetic-desktop-full

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    python3-numpy \
    python3-scipy \
    python3-opencv \
    build-essential \
    cmake \
    git \
    ros-noetic-moveit \
    ros-noetic-pcl-ros \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-octomap \
    ros-noetic-octomap-server \
    ros-noetic-octomap-ros \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    libeigen3-dev \
    libpcl-dev \
    x11-apps \
    libx11-6 \
    libxext6 \
    libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

# Set display environment variables
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Add X11 socket volume
VOLUME /tmp/.X11-unix:/tmp/.X11-unix:rw

# Create workspace directory
WORKDIR /root/comp0250_s25_labs

# Set environment variables
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /root/comp0250_s25_labs/devel/setup.bash" >> ~/.bashrc

# Set entrypoint
# Add xhost permissions
RUN echo 'if [ -z "$DISPLAY" ]; then\n\
    export DISPLAY=:0\n\
fi' >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]
