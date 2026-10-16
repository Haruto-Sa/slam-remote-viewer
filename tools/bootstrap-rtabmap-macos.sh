#!/usr/bin/env bash
# Isolated, pinned RTAB-Map + camera-side sender build for Apple Silicon.
set -euo pipefail

if [[ "${1:-}" == --help ]]; then
    echo "Usage: bash tools/bootstrap-rtabmap-macos.sh"
    echo "Requires native arm64 macOS, Xcode command-line tools, Git and conda."
    echo "SLAM_RTAB_WORK_ROOT: dedicated temporary directory (default /private/tmp/slam-remote-viewer-rtabmap)"
    echo "SLAM_CONDA_EXE: conda executable (default /opt/anaconda3/bin/conda)"
    echo "SLAM_BUILD_JOBS: parallel build jobs (default 4)"
    echo "Downloads dependencies/source, builds, installs under the work root, then runs tests."
    exit 0
fi
[[ $# == 0 ]] || { echo "error: unexpected arguments; use --help" >&2; exit 2; }

fail() { echo "error: $*" >&2; exit 1; }
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
work_root="${SLAM_RTAB_WORK_ROOT:-/private/tmp/slam-remote-viewer-rtabmap}"
conda_exe="${SLAM_CONDA_EXE:-/opt/anaconda3/bin/conda}"
jobs="${SLAM_BUILD_JOBS:-4}"
revision=d069becc724905d6465a8758c40dae88e13bb9e8

[[ "$(uname -s)" == Darwin && "$(uname -m)" == arm64 ]] || fail "run in native arm64 macOS, not Rosetta"
[[ "$jobs" =~ ^[1-9][0-9]*$ && ${#jobs} -le 2 && "$jobs" -le 32 ]] || fail "SLAM_BUILD_JOBS must be 1..32"
[[ "$work_root" == /private/tmp/* || "$work_root" == /tmp/* ]] || fail "work root must be an explicit temporary subdirectory"
[[ "$work_root" != *'/../'* && "$work_root" != */.. && "$work_root" != *';'* && "$work_root" != *$'\n'* ]] || fail "invalid work root"
[[ -x "$conda_exe" ]] || fail "conda executable not found: $conda_exe"
sdk="$(xcrun --show-sdk-path)"
[[ -f "$sdk/usr/include/pthread.h" && -f "$sdk/usr/lib/libpthread.tbd" ]] || fail "macOS SDK pthread files not found"
mkdir -p "$work_root"
work_root="$(cd "$work_root" && pwd -P)"
[[ "$work_root" == /private/tmp/* ]] || fail "work root resolves outside a temporary subdirectory"
deps="$work_root/env"
source_dir="$work_root/rtabmap"
build_dir="$work_root/build"
install_dir="$work_root/install"
adapter_dir="$work_root/sender"
for managed_path in "$deps" "$source_dir" "$build_dir" "$install_dir" "$adapter_dir"; do
    [[ ! -L "$managed_path" ]] || fail "managed path must not be a symlink: $managed_path"
done

# Never reset, overwrite, or patch an existing source checkout.
if [[ -e "$source_dir" ]]; then
    [[ -d "$source_dir/.git" ]] || fail "existing source directory is not a Git checkout"
    [[ "$(git -C "$source_dir" rev-parse HEAD)" == "$revision" ]] || fail "existing source is not the pinned revision; choose a fresh work root"
    [[ -z "$(git -C "$source_dir" status --porcelain)" ]] || fail "existing source contains changes; choose a fresh work root"
else
    git clone --depth 1 --branch 0.23.8 https://github.com/introlab/rtabmap.git "$source_dir"
    [[ "$(git -C "$source_dir" rev-parse HEAD)" == "$revision" ]] || fail "downloaded source revision does not match pin"
fi

packages=("cmake=3.31.8" "opencv=4.13.0" "pcl=1.15.1" "eigen=3.4.0" "gtsam=4.2.0" "zlib=1.3.2" sqlite ninja)
if [[ -e "$deps" ]]; then
    [[ -f "$deps/conda-meta/history" ]] || fail "existing env is not a conda environment"
    CONDA_SUBDIR=osx-arm64 "$conda_exe" install -y -p "$deps" --override-channels -c conda-forge "${packages[@]}"
else
    CONDA_SUBDIR=osx-arm64 "$conda_exe" create -y -p "$deps" --override-channels -c conda-forge "${packages[@]}"
fi
# Exact transitive package URLs are evidence of this install, not a claim that
# future solver runs necessarily choose identical build variants.
"$conda_exe" list -p "$deps" --explicit > "$work_root/conda-explicit.txt"
export PATH="$deps/bin:/usr/bin:/bin:/usr/sbin:/sbin"

cmake -S "$source_dir" -B "$build_dir" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_C_COMPILER=/usr/bin/clang -DCMAKE_CXX_COMPILER=/usr/bin/clang++ \
    -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_SYSROOT="$sdk" \
    -DCMAKE_PREFIX_PATH="$deps" -DCMAKE_INSTALL_PREFIX="$install_dir" \
    -DCMAKE_SHARED_LINKER_FLAGS="-L\"$deps/lib\"" -DCMAKE_BUILD_RPATH="$deps/lib" \
    -DOpenCV_DIR="$deps/lib/cmake/opencv4" \
    -DPTHREADS_INCLUDE_DIR="$sdk/usr/include" -DPTHREADS_LIBRARY="$sdk/usr/lib/libpthread.tbd" \
    -DSQLite3_INCLUDE_DIR="$deps/include" -DSQLite3_LIBRARY="$deps/lib/libsqlite3.dylib" \
    -DBUILD_APP=OFF -DBUILD_TOOLS=OFF -DBUILD_EXAMPLES=OFF -DWITH_QT=OFF \
    -DWITH_GTSAM=ON -DWITH_G2O=OFF -DWITH_TORO=OFF -DWITH_VERTIGO=OFF -DWITH_MADGWICK=OFF \
    -DWITH_OPENMP=OFF -DWITH_PDAL=OFF -DWITH_ORB_OCTREE=OFF -DWITH_ORB_SLAM=OFF \
    -DWITH_FREENECT=OFF -DWITH_FREENECT2=OFF -DWITH_OPENNI=OFF -DWITH_OPENNI2=OFF \
    -DWITH_REALSENSE=OFF -DWITH_REALSENSE2=OFF -DWITH_REALSENSE_SLAM=OFF \
    -DWITH_OCTOMAP=OFF -DWITH_K4A=OFF -DWITH_K4W2=OFF -DWITH_DC1394=OFF \
    -DWITH_FLYCAPTURE2=OFF -DWITH_ZED=OFF -DWITH_ZEDOC=OFF -DWITH_MYNTEYE=OFF \
    -DWITH_MRPT=OFF -DWITH_POINTMATCHER=OFF -DWITH_OPENGV=OFF -DWITH_FASTCV=OFF
cmake --build "$build_dir" --parallel "$jobs"
cmake --install "$build_dir"

cmake -S "$repo_root/sender/slam" -B "$adapter_dir" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release -DSLAM_ENABLE_RTABMAP=ON -DSLAM_ENABLE_ORB_SLAM3=OFF \
    -DCMAKE_CXX_COMPILER=/usr/bin/clang++ -DCMAKE_OBJCXX_COMPILER=/usr/bin/clang++ \
    -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_SYSROOT="$sdk" \
    -DCMAKE_PREFIX_PATH="$install_dir;$deps" -DCMAKE_BUILD_RPATH="$deps/lib" \
    -DCMAKE_EXE_LINKER_FLAGS="-L\"$deps/lib\""
cmake --build "$adapter_dir" --parallel "$jobs"
ctest --test-dir "$adapter_dir" --output-on-failure --timeout 180
"$adapter_dir/rtabmap_rgbd_sender" --help
echo "RTAB-Map sender ready: $adapter_dir/rtabmap_rgbd_sender"
echo "Package manifest: $work_root/conda-explicit.txt"
echo "No physical camera or Unity rendering was tested by this bootstrap."
