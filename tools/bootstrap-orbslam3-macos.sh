#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
lock_file="${repo_root}/sender/slam/orbslam3/dependencies.lock.sh"
patch_directory="${repo_root}/sender/slam/orbslam3/patches"

# shellcheck source=/dev/null
source "${lock_file}"

work_root="${SLAM_ORB_WORK_ROOT:-/private/tmp/slam-remote-viewer-orbslam3}"
conda_exe="${SLAM_CONDA_EXE:-/opt/anaconda3/bin/conda}"
jobs="${SLAM_BUILD_JOBS:-$(sysctl -n hw.logicalcpu 2>/dev/null || echo 4)}"
env_prefix="${work_root}/env"
source_root="${work_root}/src"
build_root="${work_root}/build"
install_prefix="${work_root}/install"

fail() {
    echo "error: $*" >&2
    exit 1
}

[[ "$(uname -s)" == "Darwin" ]] || fail "this bootstrap supports macOS only"
[[ "$(uname -m)" == "arm64" ]] || fail "expected a native arm64 shell"
[[ -x "${conda_exe}" ]] || fail "arm64 conda executable not found: ${conda_exe}"
[[ "${work_root}" == /private/tmp/* || "${work_root}" == /tmp/* ]] || \
    fail "SLAM_ORB_WORK_ROOT must be an explicit temporary directory"

mkdir -p "${source_root}" "${build_root}" "${install_prefix}"

if [[ ! -x "${env_prefix}/bin/cmake" ]]; then
    "${conda_exe}" create -y -p "${env_prefix}" \
        "cmake=${CONDA_CMAKE_VERSION}" \
        "eigen=${CONDA_EIGEN_VERSION}" \
        "opencv=${CONDA_OPENCV_VERSION}" \
        "boost-cpp=${CONDA_BOOST_VERSION}" \
        "openssl=${CONDA_OPENSSL_VERSION}" \
        "glew=${CONDA_GLEW_VERSION}" \
        pkg-config make
fi

if [[ ! -e "${env_prefix}/include/GL/glew.h" ]]; then
    "${conda_exe}" install -y -p "${env_prefix}" \
        "glew=${CONDA_GLEW_VERSION}" "openssl=${CONDA_OPENSSL_VERSION}"
fi

export PATH="${env_prefix}/bin:/usr/bin:/bin:/usr/sbin:/sbin"
export CMAKE_PREFIX_PATH="${install_prefix};${env_prefix}"
export PKG_CONFIG_PATH="${install_prefix}/lib/pkgconfig:${env_prefix}/lib/pkgconfig"

clone_at_revision() {
    local repository="$1"
    local revision="$2"
    local destination="$3"

    if [[ ! -d "${destination}/.git" ]]; then
        git clone --filter=blob:none "${repository}" "${destination}"
    fi
    if ! git -C "${destination}" cat-file -e "${revision}^{commit}" 2>/dev/null; then
        git -C "${destination}" fetch --depth 1 origin "${revision}"
    fi
    git -C "${destination}" checkout --detach "${revision}"
    [[ "$(git -C "${destination}" rev-parse HEAD)" == "${revision}" ]] || \
        fail "revision verification failed for ${destination}"
}

clone_at_revision "${PANGOLIN_REPOSITORY}" "${PANGOLIN_REVISION}" "${source_root}/Pangolin"
clone_at_revision "${ORB_SLAM3_REPOSITORY}" "${ORB_SLAM3_REVISION}" "${source_root}/ORB_SLAM3"

cmake -S "${source_root}/Pangolin" -B "${build_root}/pangolin" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON \
    -DCMAKE_INSTALL_PREFIX="${install_prefix}" \
    -DCMAKE_OSX_ARCHITECTURES=arm64 \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TOOLS=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_PANGOLIN_PYTHON=OFF \
    -DBUILD_PANGOLIN_FFMPEG=OFF \
    -DBUILD_PANGOLIN_LIBJPEG=OFF \
    -DBUILD_PANGOLIN_LIBPNG=OFF \
    -DBUILD_PANGOLIN_LIBOPENEXR=OFF \
    -DBUILD_PANGOLIN_LIBRAW=OFF \
    -DBUILD_PANGOLIN_LIBTIFF=OFF \
    -DBUILD_PANGOLIN_LIBUVC=OFF \
    -DBUILD_PANGOLIN_LZ4=OFF \
    -DBUILD_PANGOLIN_OPENNI=OFF \
    -DBUILD_PANGOLIN_OPENNI2=OFF \
    -DBUILD_PANGOLIN_REALSENSE=OFF \
    -DBUILD_PANGOLIN_REALSENSE2=OFF \
    -DBUILD_PANGOLIN_ZSTD=OFF
cmake --build "${build_root}/pangolin" --parallel "${jobs}"
cmake --install "${build_root}/pangolin"

orb_source="${source_root}/ORB_SLAM3"
for patch_file in "${patch_directory}"/*.patch; do
    if git -C "${orb_source}" apply --unidiff-zero --reverse --check "${patch_file}" >/dev/null 2>&1; then
        echo "ORB-SLAM3 patch already applied: $(basename "${patch_file}")"
    elif git -C "${orb_source}" apply --unidiff-zero --check "${patch_file}" >/dev/null 2>&1; then
        git -C "${orb_source}" apply --unidiff-zero "${patch_file}"
    else
        fail "ORB-SLAM3 source has unexpected changes; use a fresh temporary work root"
    fi
done

cmake -S "${orb_source}/Thirdparty/DBoW2" -B "${build_root}/dbow2" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON \
    -DCMAKE_OSX_ARCHITECTURES=arm64
cmake --build "${build_root}/dbow2" --parallel "${jobs}"

cmake -S "${orb_source}" -B "${build_root}/orbslam3" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON \
    -DCMAKE_OSX_ARCHITECTURES=arm64 \
    -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}" \
    -DOpenCV_DIR="${env_prefix}/lib/cmake/opencv4" \
    -DPangolin_DIR="${install_prefix}/lib/cmake/Pangolin"
cmake --build "${build_root}/orbslam3" --parallel "${jobs}"

"${repo_root}/tools/verify-orbslam3-build.sh" \
    "${orb_source}" "${install_prefix}" "${env_prefix}"
