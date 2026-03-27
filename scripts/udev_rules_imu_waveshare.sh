#!/bin/bash

# Waveshare IMU (PL2303)

RULE_FILE="/etc/udev/rules.d/99-waveshare-imu.rules"

echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"067b\", ATTRS{idProduct}==\"2303\", SYMLINK+=\"waveshare_imu\", GROUP=\"dialout\", MODE=\"0666\"" | sudo tee $RULE_FILE

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Udev правило создано. Теперь выполните следующие шаги:"
echo "1. Отключите и снова подключите IMU устройство"
echo "2. Проверьте новую символьную ссылку:"
echo "   ls -l /dev/waveshare_imu"
echo "3. Убедитесь в правах доступа:"
echo "   ls -l /dev/tty* | grep waveshare"
echo "4. В конфигурационных файлах ROS используйте: device: \"/dev/waveshare_imu\""
echo "5. Для проверки работы устройства:"
echo "   screen /dev/waveshare_imu 115200"