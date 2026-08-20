#!/usr/bin/env bash
set -euo pipefail

# Install system, ROS 1 Noetic, and Python dependencies needed by the three
# packages in this directory: jie_map_msgs, jie_octomap, and octo_planner.
#
# Target OS: Ubuntu 20.04 (focal)
# Usage:
#   cd ~/catkin_ws/src/jie_3d_nav_ros1
#   bash install_deps_noetic.sh
#
# Notes:
# - This script installs external dependencies only. Local workspace packages
#   such as d1_bringup, d1_description, odin_ros_driver, and odin_costmap
#   still need to be copied into the same ROS 1 workspace if you want to launch
#   the full robot stack.
# - jie_octomap uses the C++ Open3D package via find_package(Open3D). Ubuntu
#   repositories may not provide libopen3d-dev on every machine. The script
#   detects that case and prints a clear follow-up action.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
UBUNTU_CODENAME="$(. /etc/os-release && echo "${VERSION_CODENAME}")"

if [[ "${UBUNTU_CODENAME}" != "focal" ]]; then
  echo "ERROR: this script targets Ubuntu 20.04 (focal), current codename: ${UBUNTU_CODENAME}" >&2
  exit 1
fi

if [[ "${ROS_DISTRO}" != "noetic" ]]; then
  echo "ERROR: this script targets ROS 1 Noetic on Ubuntu 20.04, ROS_DISTRO=${ROS_DISTRO}" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  curl \
  gnupg \
  lsb-release \
  software-properties-common

if [[ ! -f /etc/apt/sources.list.d/ros-latest.list ]]; then
  sudo add-apt-repository universe -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu ${UBUNTU_CODENAME} main" \
    | sudo tee /etc/apt/sources.list.d/ros-latest.list >/dev/null
fi

sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-catkin-tools \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  python3-vtk7 \
  python3-numpy \
  python3-pil \
  python3-pyqt5 \
  python3-yaml \
  libeigen3-dev \
  liboctomap-dev \
  libopencv-dev \
  libqt5gui5 \
  libqt5opengl5 \
  libqt5widgets5 \
  libqt5x11extras5 \
  libtinyxml2-dev \
  libc++1 \
  libc++abi1 \
  libc++-dev \
  libc++abi-dev \
  libvtk7-dev \
  qt5-qmake \
  qtbase5-dev \
  qtbase5-dev-tools \
  ros-${ROS_DISTRO}-roscpp \
  ros-${ROS_DISTRO}-rospy \
  ros-${ROS_DISTRO}-message-generation \
  ros-${ROS_DISTRO}-message-runtime \
  ros-${ROS_DISTRO}-geometry-msgs \
  ros-${ROS_DISTRO}-nav-msgs \
  ros-${ROS_DISTRO}-octomap \
  ros-${ROS_DISTRO}-octomap-msgs \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-rosbridge-server \
  ros-${ROS_DISTRO}-rviz \
  ros-${ROS_DISTRO}-sensor-msgs \
  ros-${ROS_DISTRO}-std-msgs \
  ros-${ROS_DISTRO}-tf2 \
  ros-${ROS_DISTRO}-tf2-geometry-msgs \
  ros-${ROS_DISTRO}-tf2-msgs \
  ros-${ROS_DISTRO}-tf2-ros \
  ros-${ROS_DISTRO}-visualization-msgs \
  ros-${ROS_DISTRO}-map-server

if apt-cache show libopen3d-dev >/dev/null 2>&1; then
  sudo apt-get install -y libopen3d-dev
else
  cat >&2 <<'EOF'
WARNING: apt package libopen3d-dev was not found.
jie_octomap builds C++ nodes with find_package(Open3D REQUIRED), so install
Open3D C++ development files before building jie_octomap.

Expected result after installation:
  Open3DConfig.cmake is visible through CMAKE_PREFIX_PATH or Open3D_DIR.

Common options:
  - install a distro/vendor package that provides libopen3d-dev, or
  - build Open3D from source and export Open3D_DIR to its CMake config path.
EOF
fi

python3 -m pip install --user --upgrade pip
python3 -m pip install --user --upgrade open3d

# Detect rosdep or rosdepc
ROSDEP_CMD="rosdep"
if command -v rosdepc >/dev/null 2>&1; then
  ROSDEP_CMD="rosdepc"
fi

if [[ "${ROSDEP_CMD}" == "rosdep" ]]; then
  if [[ ! -d /etc/ros/rosdep/sources.list.d ]]; then
    sudo rosdep init || true
  fi
  rosdep update
else
  echo "Using rosdepc (domestic mirror)..."
  rosdepc update
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"

ROSDEP_SKIP_KEYS=(
  d1_bringup
  d1_description
  odin_costmap
  odin_ros_driver
  opencv3
)

${ROSDEP_CMD} install \
  --from-paths "${SCRIPT_DIR}/jie_map_msgs" "${SCRIPT_DIR}/jie_octomap" "${SCRIPT_DIR}/octo_planner" \
  --ignore-src \
  --rosdistro "${ROS_DISTRO}" \
  --skip-keys "${ROSDEP_SKIP_KEYS[*]}" \
  -r -y


cat <<EOF

Dependency installation finished.

Next steps:
  1. Ensure these local/non-apt packages also exist in your workspace if needed:
     d1_bringup d1_description odin_ros_driver odin_costmap
  2. If CMake cannot find Open3D, install Open3D C++ dev files and set Open3D_DIR.
  3. Build from the workspace root, for example:
     cd "$(cd "${SCRIPT_DIR}/../../.." && pwd)"
     source /opt/ros/${ROS_DISTRO}/setup.bash
     catkin_make -DCMAKE_BUILD_TYPE=Release
     # or using catkin build:
     # catkin build jie_map_msgs jie_octomap octo_planner
EOF
