#!/bin/bash
# YAVUZ AUV - Kurulum Scripti
# ROS2 Jazzy + Gazebo Harmonic + Python bağımlılıkları

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$SCRIPT_DIR"

echo "============================================="
echo " YAVUZ AUV Simülasyon Kurulum Scripti"
echo " TEKNOFEST 2026 - İleri Kategori Tema 2"
echo "============================================="

# ROS2 Jazzy kurulu mu kontrol et
if ! command -v ros2 &> /dev/null; then
    echo "[HATA] ROS2 bulunamadı! Lütfen ROS2 Jazzy kurun."
    echo "  https://docs.ros.org/en/jazzy/Installation.html"
    exit 1
fi

echo "[1/5] ROS2 environment hazırlanıyor..."
source /opt/ros/jazzy/setup.bash

echo "[2/5] Sistem bağımlılıkları kuruluyor..."
sudo apt-get update -q
sudo apt-get install -y -q \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-robot-localization \
    ros-jazzy-nav2-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-nav-msgs \
    python3-pip \
    python3-numpy \
    python3-scipy \
    python3-transforms3d

echo "[3/5] Python bağımlılıkları kuruluyor..."
pip3 install --quiet \
    numpy \
    scipy \
    transforms3d \
    PyYAML \
    filterpy

echo "[4/5] Workspace derleniyor..."
cd "$WS_DIR"
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers $(nproc)

echo "[5/5] Environment setup..."
echo ""
echo "============================================="
echo " Kurulum tamamlandı!"
echo " Simülasyonu başlatmak için:"
echo "   source install/setup.bash"
echo "   ros2 launch yavuz_bringup simulation.launch.py"
echo "============================================="
