#!/usr/bin/env bash
set -euo pipefail

# This script downloads and installs the pre-compiled C++ Open3D (cxx11-abi)
# library for Ubuntu 20.04 to prevent ABI mismatch errors with ROS Noetic.
#
# It automatically detects system architecture (x86_64 vs. ARM64/aarch64).
# - x86_64: Downloads the pre-built official binary package.
# - ARM64 (aarch64): Compiles from source (since no official prebuilt ARM64 C++ release is provided).
#
# Target Path: ~/software/open3d
# Usage:
#   cd ~/catkin_ws/src/jie_3d_nav_ros1
#   bash install_open3d.sh

INSTALL_DIR="$HOME/software"
OPEN3D_VER="0.18.0"
ARCH="$(uname -m)"

echo "=== Open3D C++ Library Installation Script ==="
echo "Detected architecture: ${ARCH}"
echo "Target path: ${INSTALL_DIR}/open3d"

# 1. Check if already installed
if [ -d "${INSTALL_DIR}/open3d" ]; then
  echo "Open3D directory already exists at ${INSTALL_DIR}/open3d."
  echo "If you want to reinstall, please delete or rename it first."
  echo "For example: rm -rf ${INSTALL_DIR}/open3d"
  exit 0
fi

# 2. Create installation directory
mkdir -p "${INSTALL_DIR}"

if [[ "${ARCH}" == "x86_64" ]]; then
  # Official x86_64 C++ release download
  TAR_FILE="open3d-devel-linux-x86_64-cxx11-abi-${OPEN3D_VER}.tar.xz"
  DOWNLOAD_URL="https://github.com/isl-org/Open3D/releases/download/v${OPEN3D_VER}/${TAR_FILE}"

  echo "Downloading Open3D v${OPEN3D_VER} C++ pre-built package for x86_64..."
  echo "URL: ${DOWNLOAD_URL}"

  TEMP_DIR=$(mktemp -d)
  trap 'rm -rf "$TEMP_DIR"' EXIT

  if command -v wget >/dev/null 2>&1; then
    wget --show-progress -O "${TEMP_DIR}/${TAR_FILE}" "${DOWNLOAD_URL}"
  elif command -v curl >/dev/null 2>&1; then
    curl -L -o "${TEMP_DIR}/${TAR_FILE}" "${DOWNLOAD_URL}"
  else
    echo "ERROR: Neither wget nor curl is installed. Please install one of them first." >&2
    exit 1
  fi

  echo "Extracting Open3D to ${INSTALL_DIR}..."
  tar -xf "${TEMP_DIR}/${TAR_FILE}" -C "${INSTALL_DIR}"

  # Rename the extracted folder (like open3d-devel-...) to open3d
  EXTRACTED_DIR=$(find "${INSTALL_DIR}" -maxdepth 1 -type d -name "open3d-devel-*")
  if [ -n "${EXTRACTED_DIR}" ]; then
    mv "${EXTRACTED_DIR}" "${INSTALL_DIR}/open3d"
  fi

elif [[ "${ARCH}" == "aarch64" || "${ARCH}" == "arm64" ]]; then
  # Build from source for ARM64/aarch64
  echo "WARNING: Open3D does NOT provide official pre-compiled C++ developer binaries for Linux ARM64 (aarch64)."
  echo "We must build it from source."
  echo "Note: Compiling Open3D on ARM64 devices (like Raspberry Pi or NVIDIA Jetson)"
  echo "can take 1-2 hours and requires sufficient RAM (at least 4GB or a configured swap file)."
  echo ""

  # Check if interactive
  if [ -t 0 ]; then
    read -p "Do you want to continue compiling Open3D from source? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
      echo "Installation cancelled."
      exit 1
    fi
  else
    echo "Non-interactive environment detected. Proceeding in 5 seconds..."
    sleep 5
  fi

  echo "Cloning Open3D v${OPEN3D_VER} repository..."
  SRC_DIR="/tmp/open3d_source"
  rm -rf "${SRC_DIR}"
  git clone --recursive -b "v${OPEN3D_VER}" https://github.com/isl-org/Open3D.git "${SRC_DIR}"

  echo "Installing system-level build dependencies..."
  cd "${SRC_DIR}"
  
  # Run the official dependency installer script
  if [ -f "./util/install_deps_ubuntu.sh" ]; then
    sudo ./util/install_deps_ubuntu.sh
  else
    echo "Dependency script not found, installing standard dependencies via apt..."
    sudo apt-get update && sudo apt-get install -y \
      libgl1-mesa-dev libglu1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev \
      libxcursor-dev libxi-dev xorg-dev libxxf86vm-dev libasound2-dev libpthread-stubs0-dev
  fi

  echo "Configuring and compiling Open3D..."
  mkdir build && cd build
  # We use -DGLIBCXX_USE_CXX11_ABI=ON to match ROS Noetic C++11 ABI
  # -DBUILD_GUI=OFF is recommended on embedded ARM devices to avoid heavy filament/GUI build errors
  cmake -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -DGLIBCXX_USE_CXX11_ABI=ON \
        -DBUILD_GUI=OFF \
        -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}/open3d" \
        ..

  echo "Starting build with $(nproc) cores..."
  make -j$(nproc)
  
  echo "Installing Open3D to target directory..."
  make install

  # Clean up source
  rm -rf "${SRC_DIR}"
else
  echo "ERROR: Unsupported architecture: ${ARCH}" >&2
  exit 1
fi

# 3. Success message & configuration guide
echo "============================================="
echo "Open3D v${OPEN3D_VER} C++ Library installed successfully!"
echo "Location: ${INSTALL_DIR}/open3d"
echo ""
echo "To let CMake find this installation, you can run:"
echo "  export Open3D_DIR=${INSTALL_DIR}/open3d/lib/cmake/Open3D"
echo ""
echo "Or add the following line to your ~/.bashrc:"
echo "  export Open3D_DIR=\$HOME/software/open3d/lib/cmake/Open3D"
echo "============================================="
