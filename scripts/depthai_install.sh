#!/bin/bash

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' 
SUCCESS="✅"  
TOOLS="🛠️" 
INFO="ℹ️" 
WARNING="⚠️"
ERROR="❌"

ROS2_WS="$HOME/ros2_ws"
SRC_DIR="$ROS2_WS/src"
DEPTHAI_REPO="https://github.com/luxonis/depthai-ros.git"
DEPTHAI_BRANCH="jazzy"
DEPTHAI_DIR="$SRC_DIR/depthai-ros"

# Cleanup function to handle build directory issues
clean_build_directory() {
    echo -e "${YELLOW}${WARNING} Cleaning build directory for $1...${NC}"
    rm -rf "$ROS2_WS/build/$1"
}

if [ ! -d "$ROS2_WS" ]; then
    echo -e "${RED}${ERROR} ROS 2 workspace $ROS2_WS not found!${NC}"
    echo "Please create the workspace first: mkdir -p $ROS2_WS/src"
    exit 1
fi

if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}${ERROR} ROS_DISTRO is not set!${NC}"
    echo "Please source your ROS 2 installation first"
    exit 1
fi

cd "$ROS2_WS" || exit

echo -e "${GREEN}${TOOLS} Installing required system packages...${NC}"
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-depthai \
    ros-$ROS_DISTRO-ffmpeg-image-transport-msgs \
    ros-$ROS_DISTRO-apriltag

# Clone DepthAI repository
if [ ! -d "$DEPTHAI_DIR" ]; then
    echo -e "${GREEN}${TOOLS} Cloning DepthAI repository...${NC}"
    cd "$SRC_DIR" || exit
    git clone "$DEPTHAI_REPO"
    cd "$DEPTHAI_DIR" || exit
    git checkout "$DEPTHAI_BRANCH"
    cd "$ROS2_WS" || exit
else
    echo -e "${GREEN}${INFO} DepthAI repository already exists, pulling updates...${NC}"
    cd "$DEPTHAI_DIR" || exit
    git checkout "$DEPTHAI_BRANCH"
    git pull origin "$DEPTHAI_BRANCH"
    cd "$ROS2_WS" || exit
fi

# Clean build directories for packages that might have symlink issues
clean_build_directory "depthai_ros_driver"
clean_build_directory "depthai_ros_msgs"

echo -e "${YELLOW}${WARNING} Note about package overriding:${NC}"
echo -e "${YELLOW}Some DepthAI packages may be installed in underlay workspaces.${NC}"
echo -e "${YELLOW}We'll build with --allow-overriding to ensure proper compilation.${NC}"

echo -e "${GREEN}${TOOLS} Building workspace from $PWD...${NC}"
colcon build --symlink-install \
    --packages-select \
        depthai_bridge \
        depthai_descriptions \
        depthai_examples \
        depthai_filters \
        depthai-ros \
        depthai_ros_driver \
        depthai_ros_msgs \
    --allow-overriding \
        depthai_bridge \
        depthai_descriptions \
        depthai_examples \
        depthai_filters \
        depthai-ros \
        depthai_ros_driver \
        depthai_ros_msgs \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release

# Handle build result
if [ $? -eq 0 ]; then
    echo -e "\n${BLUE}${SUCCESS} Installation complete!${NC}"
    echo -e "${BLUE}${SUCCESS} Successfully built DepthAI packages from branch $DEPTHAI_BRANCH${NC}"
    echo -e "${YELLOW}${WARNING} Remember to source the workspace:${NC}"
    echo -e "${BLUE}source $ROS2_WS/install/setup.bash${NC}"
else
    echo -e "\n${RED}${ERROR} Build failed! Trying alternative approach...${NC}"
    
    # Try building without symlink install if the first attempt fails
    echo -e "${YELLOW}${WARNING} Attempting build without symlink install...${NC}"
    clean_build_directory "depthai_ros_driver"
    colcon build \
        --packages-select depthai_ros_driver \
        --allow-overriding depthai_ros_driver \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    if [ $? -eq 0 ]; then
        echo -e "${BLUE}${SUCCESS} Successfully built depthai_ros_driver without symlinks${NC}"
    else
        echo -e "${RED}${ERROR} Build still failing. Please check the error messages above.${NC}"
        exit 1
    fi
fi