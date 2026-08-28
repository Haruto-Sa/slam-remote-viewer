#!/bin/sh

set -u

warnings=0
missing=0

section() {
    printf '\n[%s]\n' "$1"
}

fact() {
    printf '%-24s %s\n' "$1" "$2"
}

warn() {
    printf 'WARNING: %s\n' "$1"
    warnings=$((warnings + 1))
}

missing_tool() {
    printf 'MISSING: %s\n' "$1"
    missing=$((missing + 1))
}

command_version() {
    tool_name=$1
    shift
    if command -v "$tool_name" >/dev/null 2>&1; then
        tool_path=$(command -v "$tool_name")
        tool_version=$("$@" 2>/dev/null | sed -n '1p')
        fact "$tool_name" "$tool_path${tool_version:+ ($tool_version)}"
    else
        missing_tool "$tool_name"
    fi
}

section "Host"
host_arch=$(uname -m)
fact "architecture" "$host_arch"

if command -v sw_vers >/dev/null 2>&1; then
    fact "macOS" "$(sw_vers -productVersion) ($(sw_vers -buildVersion))"
else
    warn "sw_vers is unavailable; this workflow targets macOS"
fi

translated="unknown"
if command -v sysctl >/dev/null 2>&1; then
    translated=$(sysctl -in sysctl.proc_translated 2>/dev/null || printf '0')
fi
fact "Rosetta process" "$translated"

case "$host_arch:$translated" in
    arm64:0) canonical_prefix="/opt/homebrew" ;;
    x86_64:1) canonical_prefix="/opt/homebrew" ;;
    x86_64:0) canonical_prefix="/usr/local" ;;
    *) canonical_prefix="unknown" ;;
esac
fact "expected brew prefix" "$canonical_prefix"

section "Build tools"
command_version clang clang --version
command_version cmake cmake --version
command_version pkg-config pkg-config --version
command_version git git --version
command_version rustc rustc --version
command_version cargo cargo --version

if command -v xcode-select >/dev/null 2>&1; then
    developer_dir=$(xcode-select -p 2>/dev/null || true)
    if [ -n "$developer_dir" ]; then
        fact "developer directory" "$developer_dir"
    else
        missing_tool "Xcode Command Line Tools"
    fi
fi

section "Homebrew architecture"
if command -v brew >/dev/null 2>&1; then
    brew_path=$(command -v brew)
    brew_prefix=$(brew --prefix 2>/dev/null || printf 'unknown')
    fact "brew" "$brew_path"
    fact "brew prefix" "$brew_prefix"
    if [ "$canonical_prefix" != "unknown" ] && [ "$brew_prefix" != "$canonical_prefix" ]; then
        warn "Homebrew prefix '$brew_prefix' does not match native host expectation '$canonical_prefix'"
    fi

    for brewed_tool in cmake pkg-config; do
        if command -v "$brewed_tool" >/dev/null 2>&1; then
            binary_path=$(command -v "$brewed_tool")
            binary_kind=$(file "$binary_path" 2>/dev/null || true)
            fact "$brewed_tool binary" "$binary_kind"
            if [ "$host_arch" = "arm64" ] && printf '%s' "$binary_kind" | grep -q 'x86_64'; then
                warn "$brewed_tool is x86_64 while the shell is arm64"
            fi
        fi
    done
else
    missing_tool "Homebrew"
fi

section "ORB-SLAM3 dependencies"
if command -v pkg-config >/dev/null 2>&1; then
    for package_name in opencv4 eigen3 libzmq; do
        if package_version=$(pkg-config --modversion "$package_name" 2>/dev/null); then
            fact "$package_name" "$package_version"
        else
            missing_tool "$package_name (pkg-config)"
        fi
    done
fi

pangolin_found="no"
for pangolin_root in /opt/homebrew /usr/local; do
    if [ -e "$pangolin_root/lib/cmake/Pangolin/PangolinConfig.cmake" ]; then
        fact "Pangolin" "$pangolin_root/lib/cmake/Pangolin/PangolinConfig.cmake"
        pangolin_found="yes"
    fi
done
if [ "$pangolin_found" = "no" ]; then
    missing_tool "Pangolin CMake package"
fi

section "Camera inventory"
if command -v system_profiler >/dev/null 2>&1; then
    camera_output=$(system_profiler SPCameraDataType 2>/dev/null || true)
    if [ -n "$camera_output" ]; then
        printf '%s\n' "$camera_output"
    else
        warn "system_profiler reported no camera; verify hardware visibility and privacy settings"
    fi
else
    missing_tool "system_profiler"
fi

section "Summary"
fact "warnings" "$warnings"
fact "missing requirements" "$missing"

if [ "$warnings" -gt 0 ] || [ "$missing" -gt 0 ]; then
    printf '%s\n' "Host is not ready for a reproducible ORB-SLAM3 build. See docs/slam-host.md."
    exit 1
fi

printf '%s\n' "Host prerequisites are internally consistent."
