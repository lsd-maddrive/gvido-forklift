#!/bin/bash

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' 

CAM="📷"       
TOOLS="🛠️"     
SUCCESS="✅"   
WARNING="⚠️"   
INFO="ℹ️"      
USB="🔌"       
USER="👤"      

echo -e "${GREEN}${TOOLS} Setting up udev rules for OAK-D Pro camera...${NC}"

echo -e "${BLUE}${CAM} 1. Adding udev rules...${NC}"
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
echo -e "${GREEN}${SUCCESS} Rules created: /etc/udev/rules.d/80-movidius.rules${NC}"

echo -e "${BLUE}${CAM} Updating udev rules...${NC}"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo -e "\n${BLUE}${USER} 2. Adding current user to dialout group...${NC}"
sudo usermod -a -G dialout $USER
echo -e "${GREEN}${SUCCESS} User $USER added to dialout group${NC}"

echo -e "\n${YELLOW}${WARNING} 3. Device reconnection required:${NC}"
echo -e "${YELLOW}${USB} - Disconnect OAK-D camera from USB${NC}"
echo -e "${YELLOW}${USB} - Restart udev service${NC}"
sudo service udev restart
echo -e "${YELLOW}${USB} - Reconnect the camera${NC}"

echo -e "\n${GREEN}${SUCCESS} Setup completed!${NC}"
echo -e "${YELLOW}${WARNING} To apply changes you need to:${NC}"
echo -e "${YELLOW}1. Log out and log back in${NC}"
echo -e "${YELLOW}2. Check camera connection with command:${NC}"
echo -e "   ${BLUE}lsusb | grep 03e7${NC}"
echo -e "${YELLOW}3. Check access permissions:${NC}"
echo -e "   ${BLUE}ls -l /dev/bus/usb/*/* | grep 03e7${NC}"