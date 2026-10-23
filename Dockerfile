# syntax=docker/dockerfile:1
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
    TORCH_CUDA_ARCH_LIST="10.0" \
    PIP_BREAK_SYSTEM_PACKAGES=1

# 1. 系統相依套件與 ROS 2 套件安裝
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-opencv \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-rviz2 \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-rosbag2 \
    ros-jazzy-rosbag2-cpp \
    libpcl-dev \
    libopencv-dev \
    curl jq wget ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# 2. Python 依賴統一安裝（避免重複安裝導致 numpy 被覆蓋）
RUN pip3 install --no-cache-dir \
    numpy==1.23.5 \
    "pandas<2.1" \
    tqdm \
    gnupg \
    pycryptodomex \
    rosbags \
    openvino \
    && find /usr/local/lib/python*/ -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true

# 3. Intel GPU 運算驅動鏈：本地 packages 優先，不存在或為空時自動透過 GitHub API 下載
RUN --mount=type=bind,source=packages,target=/tmp/packages \
    mkdir -p /tmp/intel_install \
    && if ls /tmp/packages/*.deb 1> /dev/null 2>&1; then \
           echo "[+] 偵測到本地 packages 目錄中有 .deb，優先使用本地安裝包..." \
           && cp /tmp/packages/*.deb /tmp/intel_install/; \
       else \
           echo "[-] 本地 packages 目錄為空，透過 GitHub API 自動抓取最新驅動鏈..." \
           && IGC_URLS=$(curl -s https://api.github.com/repos/intel/intel-graphics-compiler/releases/latest \
               | jq -r '.assets[]? | select(.name | test("intel-igc-(core|opencl).*amd64\\.deb$")) | .browser_download_url') \
           && for url in $IGC_URLS; do \
                 echo "下載 IGC: $url" && wget -q -P /tmp/intel_install "$url"; \
              done \
           && NEO_URLS=$(curl -s https://api.github.com/repos/intel/compute-runtime/releases/latest \
               | jq -r '.assets[]? | select(.name | test("(intel-opencl-icd|libigdgmm12).*amd64\\.deb$")) | .browser_download_url') \
           && for url in $NEO_URLS; do \
                 echo "下載 NEO: $url" && wget -q -P /tmp/intel_install "$url"; \
              done; \
       fi \
    && if ls /tmp/intel_install/*.deb 1> /dev/null 2>&1; then \
           (dpkg -i /tmp/intel_install/*.deb || apt-get update && apt-get install -f -y --no-install-recommends); \
       else \
           echo "[!] 警告: 未找到任何 Intel deb 安裝包，跳過安裝"; \
       fi \
    && rm -rf /tmp/intel_install \
    && echo "/usr/lib/wsl/lib" > /etc/ld.so.conf.d/wsl.conf \
    && ldconfig \
    && rm -f /etc/ld.so.preload \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /usr/share/doc/* /usr/share/man/*

WORKDIR /root/midterm

# 設定 ROS 2 環境自動載入
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]