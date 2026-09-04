#!/usr/bin/env bash

set -euo pipefail

adapter="${1:?usage: repeat-test-orbslam3-shutdown.sh ADAPTER ORB_SOURCE TUM_SEQUENCE [REPETITIONS] [OUTPUT_DIR]}"
orb_source="${2:?missing ORB-SLAM3 source directory}"
sequence_dir="${3:?missing extracted TUM RGB-D sequence directory}"
repetitions="${4:-10}"
output_dir="${5:-/private/tmp/slam-remote-viewer-orbslam3/shutdown-check}"

vocabulary="${orb_source}/Vocabulary/ORBvoc.txt"
settings="${orb_source}/Examples/Monocular/TUM1.yaml"

for required_path in "${adapter}" "${vocabulary}" "${settings}" "${sequence_dir}/rgb.txt"; do
    [[ -e "${required_path}" ]] || {
        echo "error: missing shutdown-test input: ${required_path}" >&2
        exit 1
    }
done

[[ "$(uname -s)" == "Darwin" && "$(uname -m)" == "arm64" ]] || {
    echo "error: shutdown test requires native Apple Silicon macOS" >&2
    exit 1
}

[[ "${repetitions}" =~ ^[1-9][0-9]*$ ]] || {
    echo "error: REPETITIONS must be a positive integer" >&2
    exit 1
}

mkdir -p "${output_dir}"
for ((run = 1; run <= repetitions; ++run)); do
    log_file="${output_dir}/run-${run}.log"
    echo "ORB-SLAM3 shutdown check ${run}/${repetitions}"
    "${adapter}" "${vocabulary}" "${settings}" "${sequence_dir}" 2>&1 |
        tee "${log_file}"
    grep -q "ORB-SLAM3 pose adapter replay passed:" "${log_file}" || {
        echo "error: run ${run} did not report a successful replay" >&2
        exit 1
    }
done

echo "ORB-SLAM3 shutdown check passed: ${repetitions} clean runs"
