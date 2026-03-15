FROM osrf/ros:jazzy-desktop

# 1. 替换为中科大 (USTC) 源
RUN sed -i 's/archive.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list.d/ubuntu.sources && \
    sed -i 's/security.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list.d/ubuntu.sources

# 2. 核心修复：直接使用 signed-by 指向现有的密钥或忽略验证来通过初次握手
# 我们直接使用清华/中科大通用的 key 下载指令，并确保存储到正确位置
RUN apt-get update && apt-get install -y curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.ustc.edu.cn/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list

# 3. 安装依赖 (使用常规 apt-get，增加缓存清理)
# 注意：我们将 update 和 install 放在同一层，这是 Docker 最佳实践
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-ros-gz \
    ros-jazzy-behaviortree-cpp \
    ros-jazzy-xacro \
    python3-opencv \
    build-essential \
    cmake \
    gdb \
    gdbserver \
    rsync \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# 4. 设置工作目录与环境
WORKDIR /ros2_ws
ENV GZ_SIM_RESOURCE_PATH=/ros2_ws/install/share
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc