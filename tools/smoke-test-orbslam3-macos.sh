#!/usr/bin/env bash

set -euo pipefail

orb_source="${1:?usage: smoke-test-orbslam3-macos.sh ORB_SOURCE TUM_SEQUENCE [OUTPUT_DIR]}"
sequence_dir="${2:?missing extracted TUM RGB-D sequence directory}"
output_dir="${3:-/private/tmp/slam-remote-viewer-orbslam3/smoke}"

mono_tum="${orb_source}/Examples/Monocular/mono_tum"
vocabulary="${orb_source}/Vocabulary/ORBvoc.txt"
settings="${orb_source}/Examples/Monocular/TUM1.yaml"

for required_path in "${mono_tum}" "${vocabulary}" "${settings}" "${sequence_dir}/rgb.txt"; do
    [[ -e "${required_path}" ]] || {
        echo "error: missing smoke-test input: ${required_path}" >&2
        exit 1
    }
done

[[ "$(uname -s)" == "Darwin" && "$(uname -m)" == "arm64" ]] || {
    echo "error: smoke test requires native Apple Silicon macOS" >&2
    exit 1
}

mkdir -p "${output_dir}"
started_at="${SECONDS}"
(
    cd "${output_dir}"
    "${mono_tum}" "${vocabulary}" "${settings}" "${sequence_dir}" \
        2>&1 | tee mono_tum.log
)
elapsed_seconds="$((SECONDS - started_at))"

trajectory="${output_dir}/KeyFrameTrajectory.txt"
[[ -s "${trajectory}" ]] || {
    echo "error: tracking did not produce a key-frame trajectory" >&2
    exit 1
}

keyframes="$(wc -l < "${trajectory}" | tr -d ' ')"
[[ "${keyframes}" -gt 0 ]] || {
    echo "error: tracking produced no key frames" >&2
    exit 1
}

echo "ORB-SLAM3 monocular smoke test passed: ${keyframes} key frames in ${elapsed_seconds}s"
