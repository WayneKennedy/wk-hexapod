# Hexapod ROS 2 Jazzy Container
# Base image with ROS 2 Jazzy for arm64
FROM ros:jazzy-ros-base

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Add Raspberry Pi apt repository for Pi Camera support
RUN apt-get update && apt-get install -y --no-install-recommends \
    gnupg \
    curl \
    && curl -fsSL http://archive.raspberrypi.com/debian/raspberrypi.gpg.key | gpg --dearmor -o /usr/share/keyrings/raspberrypi-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/raspberrypi-archive-keyring.gpg] http://archive.raspberrypi.com/debian/ bookworm main" > /etc/apt/sources.list.d/raspi.list \
    && rm -rf /var/lib/apt/lists/*

# Install libjpeg62-turbo from Debian (required by rpicam-apps)
RUN curl -fsSL http://ftp.debian.org/debian/pool/main/libj/libjpeg-turbo/libjpeg62-turbo_2.1.5-2_arm64.deb -o /tmp/libjpeg62-turbo.deb \
    && dpkg -i /tmp/libjpeg62-turbo.deb \
    && rm /tmp/libjpeg62-turbo.deb

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Python and development tools
    python3-pip \
    python3-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    # I2C and SPI tools
    i2c-tools \
    python3-smbus \
    # Build essentials (needed for dlib compilation)
    build-essential \
    cmake \
    git \
    # Pi Camera support - C libraries only (Python bindings via pip)
    # Note: Can't use python3-picamera2 from Pi repo - it requires Python 3.11
    libcamera0.3 \
    libcamera-ipa \
    libcap-dev \
    # Face recognition dependencies (dlib requires these)
    libopenblas-dev \
    liblapack-dev \
    libx11-dev \
    # Debugging tools
    vim \
    htop \
    && rm -rf /var/lib/apt/lists/*

# Install Python hardware libraries
# Using --break-system-packages since we're in a container
# Note: lgpio is required for Pi 5 GPIO support (gpiozero backend)
RUN pip3 install --break-system-packages \
    lgpio \
    gpiozero \
    spidev \
    rpi-ws281x \
    mpu6050-raspberrypi \
    numpy

# Build libcamera Python bindings from source (Pi repo version requires Python 3.11)
# This takes ~10 minutes to build
RUN apt-get update && apt-get install -y --no-install-recommends \
    meson \
    ninja-build \
    pkg-config \
    libyaml-dev \
    python3-yaml \
    python3-ply \
    python3-jinja2 \
    pybind11-dev \
    libudev-dev \
    libevent-dev \
    libdrm-dev \
    libpisp-dev \
    && rm -rf /var/lib/apt/lists/*

# Clone and build libcamera with Python bindings from Raspberry Pi fork
# Using their fork which has Pi 5 (PiSP) support
RUN git clone --depth 1 https://github.com/raspberrypi/libcamera.git /tmp/libcamera && \
    cd /tmp/libcamera && \
    meson setup build \
        -Dpipelines=rpi/pisp,rpi/vc4 \
        -Dipas=rpi/pisp,rpi/vc4 \
        -Dpycamera=enabled \
        -Dtest=false \
        -Ddocumentation=disabled \
        -Dgstreamer=disabled \
        -Dcam=disabled \
        -Dqcam=disabled \
        -Dtracing=disabled \
        -Dlc-compliance=disabled \
        --prefix=/usr && \
    ninja -C build install && \
    ldconfig && \
    rm -rf /tmp/libcamera

# Install camera and vision libraries
RUN pip3 install --break-system-packages --ignore-installed \
    opencv-python-headless \
    pillow \
    picamera2

# Patch picamera2 for headless Docker (preview modules require pykms)
RUN echo '# Patched for headless Docker - make preview imports optional\n\
from .null_preview import NullPreview\n\
\n\
try:\n\
    from .drm_preview import DrmPreview\n\
except (ImportError, ModuleNotFoundError):\n\
    DrmPreview = None\n\
\n\
try:\n\
    from .qt_previews import QtGlPreview, QtPreview\n\
except (ImportError, ModuleNotFoundError):\n\
    QtGlPreview = None\n\
    QtPreview = None' > /usr/local/lib/python3.12/dist-packages/picamera2/previews/__init__.py

# Install face recognition library
# Note: dlib compilation takes 15-30 minutes on Pi 5
RUN pip3 install --break-system-packages \
    face_recognition

# Create workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Copy package sources (will be overridden by volume mount in dev)
COPY ros2_ws/src /ros2_ws/src

# Install ROS dependencies
RUN if [ -f /ros2_ws/src/hexapod_bringup/package.xml ]; then \
        . /opt/ros/jazzy/setup.sh && \
        rosdep update && \
        rosdep install --from-paths src --ignore-src -r -y; \
    fi

# Build workspace
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install || true

# Setup entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
