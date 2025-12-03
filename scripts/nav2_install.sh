#!/bin/bash

set -e

if [ -z "$ROS_DISTRO" ]; then
    echo "Ошибка: ROS_DISTRO не установлен. Убедитесь, что ROS установлен и sourced."
    exit 1
fi

echo "Используется ROS дистрибутив: $ROS_DISTRO"
echo ""

echo "Создание рабочей директории..."
mkdir -p ~/nav2_ws/src
cd ~/nav2_ws

echo "Клонирование Navigation2 репозитория..."
git clone https://github.com/ros-navigation/navigation2.git --branch $ROS_DISTRO ./src/navigation2

echo "Установка зависимостей через rosdep..."
rosdep install -y \
  --from-paths ./src \
  --ignore-src

echo "Сборка Nav2 (последовательная сборка)..."
colcon build \
  --symlink-install \
  --parallel-workers 1

echo ""
echo "Установка успешно завершена!"
echo ""
echo "Для использования Nav2 выполните:"
echo "source ~/nav2_ws/install/setup.bash"