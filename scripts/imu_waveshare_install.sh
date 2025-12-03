#!/bin/bash

GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' 
SUCCESS="✅"  
TOOLS="🛠️" 
INFO="ℹ️" 

ROS2_WS="$HOME/ros2_ws"
SRC_DIR="$ROS2_WS/src"
REPO="https://github.com/lsd-maddrive/imu_waveshare_ros2_driver.git"
BRANCH="main"  
DRIVER_DIR="$SRC_DIR/imu_waveshare_ros2_driver"
SERIAL_DIR="$DRIVER_DIR/serial"

if [ ! -d "$ROS2_WS" ]; then
    echo "Error: ROS 2 workspace $ROS2_WS not found!"
    echo "Please create the workspace first: mkdir -p $ROS2_WS/src"
    exit 1
fi

if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS_DISTRO is not set!"
    echo "Please source your ROS 2 installation first"
    exit 1
fi

cd "$ROS2_WS" || exit

echo -e "${GREEN}${TOOLS} Installing required system packages...${NC}"
sudo apt-get update
sudo apt-get install -y \
    libserial-dev \
    ros-$ROS_DISTRO-imu-tools

# Клонирование репозитория драйвера
if [ ! -d "$DRIVER_DIR" ]; then
    echo -e "${GREEN}${TOOLS} Cloning Waveshare IMU driver repository...${NC}"
    cd "$SRC_DIR" || exit
    git clone "$REPO"
    cd "$DRIVER_DIR" || exit
    git checkout "$BRANCH"
else
    echo -e "${GREEN}${INFO} Waveshare IMU driver repository already exists, pulling updates...${NC}"
    cd "$DRIVER_DIR" || exit
    git checkout "$BRANCH"
    git pull origin "$BRANCH"
fi

# Установка serial-библиотеки
echo -e "${GREEN}${TOOLS} Building and installing serial library...${NC}"
if [ -d "$SERIAL_DIR" ]; then
    cd "$SERIAL_DIR" || exit
    mkdir -p build && cd build
    cmake .. && make
    sudo make install
else
    echo -e "${RED}Error: serial directory not found in $DRIVER_DIR!${NC}"
    exit 1
fi

cd "$ROS2_WS" || exit

echo -e "${GREEN}${TOOLS} Building workspace from $PWD...${NC}"
colcon build --symlink-install --packages-select imu

echo -e "\n${BLUE}${SUCCESS} Installation complete!${NC}"
echo -e "${BLUE}${SUCCESS} Successfully built Waveshare IMU driver from branch $BRANCH${NC}"