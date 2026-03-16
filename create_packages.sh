#!/bin/bash
# YAVUZ AUV - Tüm package.xml ve setup.py dosyalarını oluştur

set -e

BASE="$(dirname "$0")/src"

# ═══════════════════════════════════════════════════════════════════
# yavuz_description
# ═══════════════════════════════════════════════════════════════════
cat > "$BASE/yavuz_description/package.xml" << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>yavuz_description</name>
  <version>1.0.0</version>
  <description>YAVUZ AUV URDF/SDF model tanımları</description>
  <maintainer email="l2321039010@ogr.sdu.edu.tr">Abdurrahim KAHRAMAN</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>robot_state_publisher</depend>
  <depend>joint_state_publisher</depend>
  <depend>xacro</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
EOF

cat > "$BASE/yavuz_description/CMakeLists.txt" << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(yavuz_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf config meshes DESTINATION share/${PROJECT_NAME}/)
ament_package()
EOF

# ═══════════════════════════════════════════════════════════════════
# yavuz_simulation
# ═══════════════════════════════════════════════════════════════════
cat > "$BASE/yavuz_simulation/package.xml" << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>yavuz_simulation</name>
  <version>1.0.0</version>
  <description>YAVUZ AUV Gazebo Harmonic simülasyon dünyası</description>
  <maintainer email="l2321039010@ogr.sdu.edu.tr">Abdurrahim KAHRAMAN</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>ros_gz_sim</depend>
  <depend>ros_gz_bridge</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
EOF

cat > "$BASE/yavuz_simulation/CMakeLists.txt" << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(yavuz_simulation)
find_package(ament_cmake REQUIRED)
install(DIRECTORY worlds models launch DESTINATION share/${PROJECT_NAME}/)
ament_package()
EOF

# ═══════════════════════════════════════════════════════════════════
# yavuz_control
# ═══════════════════════════════════════════════════════════════════
cat > "$BASE/yavuz_control/package.xml" << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>yavuz_control</name>
  <version>1.0.0</version>
  <description>YAVUZ AUV PID kontrolcüleri ve thruster allocator</description>
  <maintainer email="l2321039010@ogr.sdu.edu.tr">Abdurrahim KAHRAMAN</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <export><build_type>ament_python</build_type></export>
</package>
EOF

cat > "$BASE/yavuz_control/setup.py" << 'EOF'
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'yavuz_control'
setup(
    name=package_name, version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Abdurrahim KAHRAMAN',
    entry_points={
        'console_scripts': [],
    },
)
EOF

mkdir -p "$BASE/yavuz_control/resource"
touch "$BASE/yavuz_control/resource/yavuz_control"
touch "$BASE/yavuz_control/yavuz_control/__init__.py"

# ═══════════════════════════════════════════════════════════════════
# yavuz_navigation
# ═══════════════════════════════════════════════════════════════════
cat > "$BASE/yavuz_navigation/package.xml" << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>yavuz_navigation</name>
  <version>1.0.0</version>
  <description>YAVUZ AUV otonom navigasyon ve görev yürütücüsü</description>
  <maintainer email="l2316501412@ogr.sdu.edu.tr">Ahmet Kaan BAŞKAN</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>yavuz_control</depend>
  <export><build_type>ament_python</build_type></export>
</package>
EOF

cat > "$BASE/yavuz_navigation/setup.py" << 'EOF'
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'yavuz_navigation'
setup(
    name=package_name, version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Ahmet Kaan BAŞKAN',
    entry_points={
        'console_scripts': [
            'mission_node = yavuz_navigation.mission_state_machine:main',
        ],
    },
)
EOF

mkdir -p "$BASE/yavuz_navigation/resource"
touch "$BASE/yavuz_navigation/resource/yavuz_navigation"
touch "$BASE/yavuz_navigation/yavuz_navigation/__init__.py"

# ═══════════════════════════════════════════════════════════════════
# yavuz_localization
# ═══════════════════════════════════════════════════════════════════
cat > "$BASE/yavuz_localization/package.xml" << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>yavuz_localization</name>
  <version>1.0.0</version>
  <description>YAVUZ AUV EKF tabanlı su altı lokalizasyonu</description>
  <maintainer email="l2311012031@ogr.sdu.edu.tr">Fatih ŞAHİN</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>tf2_ros</depend>
  <export><build_type>ament_python</build_type></export>
</package>
EOF

cat > "$BASE/yavuz_localization/setup.py" << 'EOF'
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'yavuz_localization'
setup(
    name=package_name, version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='Fatih ŞAHİN',
    entry_points={
        'console_scripts': [
            'pose_estimator = yavuz_localization.pose_estimator:main',
        ],
    },
)
EOF

mkdir -p "$BASE/yavuz_localization/resource"
touch "$BASE/yavuz_localization/resource/yavuz_localization"
touch "$BASE/yavuz_localization/yavuz_localization/__init__.py"

# ═══════════════════════════════════════════════════════════════════
# yavuz_bringup
# ═══════════════════════════════════════════════════════════════════
cat > "$BASE/yavuz_bringup/package.xml" << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>yavuz_bringup</name>
  <version>1.0.0</version>
  <description>YAVUZ AUV ana launch dosyaları ve konfigürasyonlar</description>
  <maintainer email="l2321039010@ogr.sdu.edu.tr">Abdurrahim KAHRAMAN</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>yavuz_description</depend>
  <depend>yavuz_simulation</depend>
  <depend>yavuz_control</depend>
  <depend>yavuz_navigation</depend>
  <depend>yavuz_localization</depend>
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
EOF

cat > "$BASE/yavuz_bringup/CMakeLists.txt" << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(yavuz_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)
ament_package()
EOF

echo "✓ Tüm package.xml ve setup.py dosyaları oluşturuldu!"
