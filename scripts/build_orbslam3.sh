#!/usr/bin/env bash
# Build Pangolin v0.9.1 and ORB-SLAM3 v1.0 (upstream UZ-SLAMLab/ORB_SLAM3 v1.0-release, the version XTDrone's copy
# was) for the ROS 2 package xtdrone_orbslam3 (chapter 13). Needs the --all packages of setup_ubuntu_24_04_extras.sh.
#   ./scripts/build_orbslam3.sh          (then ./scripts/build_xtdrone_ros2.sh)
# Output: ~/.cache/xtdrone/orbslam3/{pangolin-install,ORB_SLAM3} (set XTDRONE_CACHE to move it).
# The clone is patched for Ubuntu 24.04 (C++17, OpenCV 4) and only the library is built.
set -euo pipefail

OUT="${XTDRONE_CACHE:-${HOME}/.cache/xtdrone}/orbslam3"
JOBS="${JOBS:-$(nproc)}"
mkdir -p "${OUT}"

if [[ ! -f "${OUT}/pangolin-install/lib/cmake/Pangolin/PangolinConfig.cmake" ]]; then
  echo "== Pangolin v0.9.1"
  rm -rf "${OUT}/Pangolin"
  git clone -q --depth 1 --branch v0.9.1 https://github.com/stevenlovegrove/Pangolin.git "${OUT}/Pangolin"
  cmake -S "${OUT}/Pangolin" -B "${OUT}/Pangolin/build" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${OUT}/pangolin-install" -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF \
    -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_PANGOLIN_FFMPEG=OFF >/dev/null
  cmake --build "${OUT}/Pangolin/build" -j "${JOBS}"
  cmake --install "${OUT}/Pangolin/build" >/dev/null
fi

echo "== ORB-SLAM3 v1.0"
SRC="${OUT}/ORB_SLAM3"
if [[ "$(git -C "${SRC}" describe --tags 2>/dev/null)" != "v1.0-release" ]]; then
  rm -rf "${SRC}"
  git clone -q --depth 1 --branch v1.0-release https://github.com/UZ-SLAMLab/ORB_SLAM3.git "${SRC}"
else
  git -C "${SRC}" checkout -q -- .  # undo the patches below before applying them again
fi
# C++17 (Pangolin 0.9) instead of C++11, any OpenCV 4 (Ubuntu 24.04: 4.6) instead of "OpenCV 4.4", no realsense
for f in "${SRC}/CMakeLists.txt" "${SRC}"/Thirdparty/*/CMakeLists.txt; do
  sed -i -e 's/-std=c++11/-std=c++17/g; s/-std=c++0x/-std=c++17/g' \
         -e 's/find_package(OpenCV [34]\.[0-9]*)/find_package(OpenCV 4 REQUIRED)/; s/find_package(OpenCV [34]\.[0-9]* QUIET)/find_package(OpenCV 4 REQUIRED)/' "$f"
done
sed -i 's/^find_package(realsense2)/#find_package(realsense2)/' "${SRC}/CMakeLists.txt"
# C++17 forbids ++ on a bool: LoopClosing's full-BA counter becomes an int (as in later ORB-SLAM3 forks)
sed -i 's/^\(\s*\)bool mnFullBAIdx;/\1int mnFullBAIdx;/' "${SRC}/include/LoopClosing.h"
# build only the library (the examples need realsense or ROS)
python3 - "${SRC}/CMakeLists.txt" <<'EOF'
import re, sys
p = sys.argv[1]
s = open(p).read()
cut = s.find("# Build examples")
if cut > 0:
    s = s[:cut] + "\n"
open(p, "w").write(s)
EOF
for t in DBoW2 g2o Sophus; do
  [[ -f "${SRC}/Thirdparty/${t}/CMakeLists.txt" ]] || continue
  echo "== ${t}"
  cmake -S "${SRC}/Thirdparty/${t}" -B "${SRC}/Thirdparty/${t}/build" -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF >/dev/null
  cmake --build "${SRC}/Thirdparty/${t}/build" -j "${JOBS}"
done
if [[ ! -f "${SRC}/Vocabulary/ORBvoc.txt" ]]; then
  tar -xzf "${SRC}/Vocabulary/ORBvoc.txt.tar.gz" -C "${SRC}/Vocabulary"
fi
cmake -S "${SRC}" -B "${SRC}/build" -DCMAKE_BUILD_TYPE=Release -DPangolin_DIR="${OUT}/pangolin-install/lib/cmake/Pangolin"
cmake --build "${SRC}/build" -j "${JOBS}"
ls -la "${SRC}/lib/libORB_SLAM3.so"
echo "ORB-SLAM3 built. Now run ./scripts/build_xtdrone_ros2.sh (package xtdrone_orbslam3)."
