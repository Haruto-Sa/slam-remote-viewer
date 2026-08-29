# macOS SLAM host

This document defines the host baseline for camera capture and SLAM processing.
It does not install software. Run the read-only diagnostic from the repository
root:

```bash
./tools/diagnose-slam-host.sh
```

The command exits non-zero when dependencies are missing or the toolchain mixes
architectures. Its output is safe to attach to an Issue after checking camera
names for information you do not want to share.

## Canonical architecture

Use one native architecture throughout the C++ dependency graph. An Apple
Silicon process reports `arm64` and uses the `/opt/homebrew` Homebrew prefix. A
native Intel process reports `x86_64` and uses `/usr/local`. Do not link arm64
objects against x86_64 Homebrew libraries or silently build part of the stack
under Rosetta.

The host baseline from Issue #21, reverified after the native-toolchain migration
for Issue #62, currently reports:

| Property | Observed value | Required action |
|---|---|---|
| Kernel process architecture | `arm64` | Keep terminal and build processes native; do not launch them through Rosetta |
| macOS | `15.6.1` (`24G90`) | Record changes when upgrading |
| Compiler | Apple Clang 17 | Keep C++ dependencies on the same compiler/standard library |
| Rust host | `aarch64-apple-darwin` | Keep the rustup toolchain native |
| Homebrew prefix | `/opt/homebrew` | Keep `/usr/local` Intel packages out of native build discovery |
| Homebrew pkg-config | `/opt/homebrew/bin/pkg-config` | Confirm discovered libraries are arm64 before linking |
| ZeroMQ | `/opt/homebrew/opt/zeromq/lib` (`arm64`) | Available for native tools that use the system library; the Rust Sender uses its pinned source build |

The host was migrated from an Intel/Rosetta Homebrew installation to native
Apple Silicon tooling. Use measured process and library architectures, not a
machine description, for build decisions. If this repository is moved to an
Intel Mac, rerun the diagnostic there and record that machine's results in its
Issue evidence rather than adding a fallback path to this setup.

## Expected dependency boundary

ORB-SLAM3 is the preferred primary backend. Its upstream documentation requires
a C++11 compiler, OpenCV 3 or newer, Eigen 3.1 or newer, and Pangolin; its
modified DBoW2 and g2o copies are included upstream. ROS is optional and is not
part of this repository's initial macOS path.

Pin exact revisions and confirm versions in Issue #26 rather than relying on
whatever Homebrew currently provides. Pangolin should be optional for the
headless telemetry process if the upstream build can be isolated accordingly.
Generated builds and an ORB-SLAM3 source checkout must not be committed here.

## Backend policy

ORB-SLAM3 remains the primary backend while it builds reproducibly and passes a
known-dataset smoke test. A failure is recorded with the command, compiler
diagnostic, architecture of linked libraries, and attempted fix. After three
independently diagnosed blocking compatibility failures, open a decision Issue
that compares maintained alternatives against the same camera and pose-source
contracts. Do not change Protocol v1 or Unity to accommodate a backend.

After the primary backend works, additional SLAM implementations may be added
behind the same contracts and conformance tests. Backend-specific types must not
cross into the Rust network layer.

## Licensing

Upstream ORB-SLAM3 is GPLv3. Treat distribution of a linked application as a
licensing decision, not only a build choice. Keep upstream code and patches
clearly attributable, record dependency licenses, and obtain project-owner
review before distributing binaries. Commercial closed-source use requires a
separate arrangement with the ORB-SLAM3 authors.

## Troubleshooting order

1. Confirm `uname -m` and whether the process is translated by Rosetta.
2. Confirm `brew --prefix` matches the canonical architecture.
3. Inspect `file` output for CMake, pkg-config, and eventually linked libraries.
4. Confirm OpenCV, Eigen, ZeroMQ, and Pangolin are discoverable from the same
   prefix.
5. Confirm macOS reports a camera before requesting application permission.
6. Only then begin the pinned ORB-SLAM3 build Issue.

## Reproducible Apple Silicon ORB-SLAM3 build

Issue #26 provides an isolated build that does not use the Intel Homebrew
libraries under `/usr/local`. It creates an osx-arm64 Conda environment and
keeps all upstream sources, patches, and build products under `/private/tmp`:

```bash
./tools/bootstrap-orbslam3-macos.sh
```

The dependency revisions and package versions are recorded in
`sender/slam/orbslam3/dependencies.lock.sh`. The upstream ORB-SLAM3 source is
patched only in the temporary checkout. The patch removes host-specific
`-march=native` flags, links the platform dylib names, and uses imported Boost
and OpenSSL targets. It does not vendor upstream code in this repository.

The final verifier rejects any ORB-SLAM3 library that is not arm64 or that
links a dependency from `/usr/local`. Override the temporary location or build
parallelism only with task-specific variables:

```bash
SLAM_ORB_WORK_ROOT=/private/tmp/my-orb-build SLAM_BUILD_JOBS=4 \
  ./tools/bootstrap-orbslam3-macos.sh
```

After the library build passes, run a monocular dataset through the upstream
`mono_*` example with `Vocabulary/ORBvoc.txt` and matching calibration. Datasets
and the expanded vocabulary remain external artifacts and must not be committed.
Live camera testing belongs to Issue #29 so dependency failures are not confused
with capture or calibration failures.
