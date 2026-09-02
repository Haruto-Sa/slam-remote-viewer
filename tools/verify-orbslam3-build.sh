#!/usr/bin/env bash

set -euo pipefail

orb_source="${1:?usage: verify-orbslam3-build.sh ORB_SOURCE PANGOLIN_PREFIX DEPENDENCY_PREFIX}"
pangolin_prefix="${2:?missing Pangolin install prefix}"
dependency_prefix="${3:?missing dependency prefix}"

require_arm64() {
    local path="$1"
    [[ -e "${path}" ]] || {
        echo "error: missing build artifact: ${path}" >&2
        return 1
    }
    file "${path}" | grep -q 'arm64' || {
        echo "error: non-arm64 build artifact: ${path}" >&2
        file "${path}" >&2
        return 1
    }
    echo "verified arm64: ${path}"
}

orb_library="${orb_source}/lib/libORB_SLAM3.dylib"
dbow_library="${orb_source}/Thirdparty/DBoW2/lib/libDBoW2.dylib"
g2o_library="${orb_source}/Thirdparty/g2o/lib/libg2o.dylib"

require_arm64 "${orb_library}"
require_arm64 "${dbow_library}"
require_arm64 "${g2o_library}"

pangolin_library="$(find "${pangolin_prefix}/lib" -maxdepth 1 -name 'libpango_core*.dylib' -print -quit)"
require_arm64 "${pangolin_library}"

opencv_library="$(find "${dependency_prefix}/lib" -maxdepth 1 -name 'libopencv_core.*.dylib' -print -quit)"
require_arm64 "${opencv_library}"

while IFS= read -r linked_path; do
    case "${linked_path}" in
        /usr/local/*)
            echo "error: ORB-SLAM3 links an Intel Homebrew path: ${linked_path}" >&2
            exit 1
            ;;
    esac
done < <(otool -L "${orb_library}" | tail -n +2 | awk '{print $1}')

echo "ORB-SLAM3 arm64 build verification passed"
