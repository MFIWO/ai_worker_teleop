#!/usr/bin/env bash
# Run inside ai_worker. Add official tactile messages without replacing existing interfaces.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
if pgrep -f '[r]os2_control_node' >/dev/null; then
  echo '[STOP] Stop the robot session before updating ROS interfaces.' >&2
  exit 1
fi
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if /usr/bin/python3 -c 'from robotis_interfaces.msg import HandPressures, TactileSensor' 2>/dev/null &&
   test -f /root/ros2_ws/install/robotis_interfaces/include/robotis_interfaces/robotis_interfaces/msg/hand_pressures.hpp; then
  echo '[HX5 INTERFACES] HandPressures and TactileSensor already installed'
  exit 0
fi
/usr/bin/python3 - "$HERE" <<'PY'
import datetime
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

workspace = Path('/root/ros2_ws')
source = workspace / 'src/robotis_interfaces'
vendor = Path(sys.argv[1]) / 'vendor/robotis_tactile_interfaces/msg'
cmake_path = source / 'CMakeLists.txt'
package_path = source / 'package.xml'
cmake = cmake_path.read_text()
package = package_path.read_text()
messages = ('HandPressures', 'TactileSensor')
for name in messages:
    expected = (vendor / (name + '.msg')).read_bytes()
    existing = source / 'msg' / (name + '.msg')
    if existing.exists() and existing.read_bytes() != expected:
        raise SystemExit(f'[STOP] Different existing message: {existing}; inspect before updating')
    entry = f'"msg/{name}.msg"'
    if entry not in cmake:
        marker = 'rosidl_generate_interfaces(${PROJECT_NAME}'
        if cmake.count(marker) != 1:
            raise SystemExit('[STOP] Unexpected interface CMake layout')
        cmake = cmake.replace(marker, marker + '\n  ' + entry)
if 'find_package(std_msgs' not in cmake:
    cmake = cmake.replace('rosidl_generate_interfaces',
                          'find_package(std_msgs REQUIRED)\n\nrosidl_generate_interfaces', 1)
# Add std_msgs to rosidl dependencies, preserving all existing message definitions.
import re
cmake, count = re.subn(r'(DEPENDENCIES[^)]*)',
    lambda m: m[0] if 'std_msgs' in m[0].split() else m[0].rstrip() + ' std_msgs\n', cmake)
if count != 1:
    raise SystemExit('[STOP] Unexpected rosidl dependency layout')
if not any(node.text == 'std_msgs' for node in ET.fromstring(package)
           if node.tag in ('depend', 'build_depend')):
    package = package.replace('</package>', '  <depend>std_msgs</depend>\n</package>')
backup = workspace / ('tactile_interface_backups/' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
backup.mkdir(parents=True)
(backup.parent / 'COLCON_IGNORE').touch()
(backup / 'COLCON_IGNORE').touch()
for folder in ('src', 'build', 'install'):
    original = workspace / folder / 'robotis_interfaces'
    if original.exists():
        shutil.copytree(original, backup / folder / 'robotis_interfaces', symlinks=True)
print('[HX5 INTERFACES] Backup:', backup, flush=True)
for name in messages:
    shutil.copy2(vendor / (name + '.msg'), source / 'msg' / (name + '.msg'))
cmake_path.write_text(cmake)
package_path.write_text(package)
PY
cd /root/ros2_ws
colcon build --base-paths /root/ros2_ws/src/robotis_interfaces \
  --packages-select robotis_interfaces --executor sequential \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF
source /root/ros2_ws/install/setup.bash
/usr/bin/python3 -c 'from robotis_interfaces.msg import HandPressures, TactileSensor; print("[HX5 INTERFACES] Both Python messages verified")'
test -f /root/ros2_ws/install/robotis_interfaces/include/robotis_interfaces/robotis_interfaces/msg/hand_pressures.hpp
ros2 interface show robotis_interfaces/msg/HandPressures
echo '[HX5 INTERFACES] Verified. No controller or motor started.'
