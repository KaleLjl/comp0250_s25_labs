FROM osrf/ros:noetic-desktop-full

# 安装必要的包
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-moveit \
    ros-noetic-pcl-ros \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# 创建工作空间目录
WORKDIR /root/comp0250_s25_labs

# 设置环境变量
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /root/comp0250_s25_labs/devel/setup.bash" >> ~/.bashrc

# 设置入口点
ENTRYPOINT ["/bin/bash"]