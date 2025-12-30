# Hexapod ROS 2 Jazzy Container
# Base image with ROS 2 Jazzy for arm64
FROM ros:jazzy-ros-base

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

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
    # Build essentials
    build-essential \
    cmake \
    git \
    # Debugging tools
    vim \
    htop \
    && rm -rf /var/lib/apt/lists/*

# Install Python hardware libraries
# Using --break-system-packages since we're in a container
RUN pip3 install --break-system-packages \
    gpiozero \
    spidev \
    rpi-ws281x \
    mpu6050-raspberrypi \
    numpy

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
