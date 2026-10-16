# RTAB-Map RGB-D backend (work in progress)

Tracked in [#108](https://github.com/Haruto-Sa/slam-remote-viewer/issues/108).
This backend is not yet release-ready. Pose-accuracy and end-to-end viewer
verification, live camera capture and independent
review remain outstanding. The default ORB-SLAM3 workflow is unchanged.

## Backend choice

RTAB-Map adds registered RGB-D mapping with depth-derived metric scale to the
existing monocular workflow. Its own frame-to-map visual odometry feeds its
mapping/loop-closure backend; this is not an ORB-SLAM wrapper or a pose mock.
It requires synchronized depth, so an ordinary browser RGB camera alone cannot
use this adapter. The smartphone RGB workflow continues to use ORB-SLAM3.

The initial integration targets upstream release
[0.23.8](https://github.com/introlab/rtabmap/releases/tag/0.23.8), commit
`d069becc724905d6465a8758c40dae88e13bb9e8`. See the upstream
[RGB-D example](https://github.com/introlab/rtabmap/blob/0.23.8/examples/RGBDMapping/main.cpp)
and [license](https://github.com/introlab/rtabmap/blob/0.23.8/LICENSE).
Its OpenCV/PCL dependencies make installation substantially larger than our
backend-neutral contract build. No conda-forge `rtabmap` package was found for
Apple Silicon in the development environment; the library is built from source.
OpenCV 5 failed configuration because the required `calib3d` component was not
available, so this integration uses OpenCV 4.

Use GTSAM optimization (`Optimizer/Strategy=2`), not TORO. The upstream build
summary identifies TORO as noncommercial; disable `WITH_TORO`, `WITH_MADGWICK`
and `WITH_VERTIGO` in this configuration. Review all transitive dependency
licenses before distributing binaries: the top-level RTAB-Map license alone
does not describe an entire dependency environment.

## Coordinate and identity contract

- Input color is rectified BGR8, with registered float32 depth in metres.
  Missing depth is zero. Non-finite and negative depth values are rejected.
- Camera local transform is explicitly identity rather than RTAB-Map's default
  optical-to-robot transform. The camera axes are right/down/forward.
- Published pose is `mapCorrection * odometry`, already camera-to-world. It is
  **not inverted**. The shared converter validates the rigid transform and emits
  normalized `[x,y,z,w]` orientation. Receiver owns conversion to Unity axes.
- A null odometry estimate emits initializing/lost without a cached pose.
  Automatic odometry resets are disabled. A reset requires a new tracker and a
  new boundary session; session-relative IDs must not cross this boundary.
- Exported points are sparse feature observations, not a dense surface or
  globally deduplicated landmarks. Identity is `(positive node ID << 20) +
  feature slot`, safely below the JSON integer limit for signed 32-bit node IDs.
  Updated optimized node poses move these observations using ordinary deltas;
  missing observations generate removals through the existing reducer.
- Point export fails above 100,000 observations instead of silently truncating
  the map. Large deltas are split into messages of at most 1 MiB, preserving
  frame/time and all operations. The full delta is validated before sending
  any chunk; more than 200,000 operations or 64 MiB of serialized chunks are
  rejected. A connection failure
  midway is still an abnormal, potentially partial session, not an atomic
  transaction. Export is
  approximately once per second of input frames, plus a final export at EOF.

## Optional build and dataset entry point

### Apple Silicon camera-side PC setup

From a native arm64 terminal, with Xcode command-line tools, Git and conda
already installed:

```sh
bash tools/bootstrap-rtabmap-macos.sh
```

If conda is installed elsewhere:

```sh
SLAM_CONDA_EXE=/absolute/path/to/conda \
  bash tools/bootstrap-rtabmap-macos.sh
```

The script uses a dedicated `/private/tmp/slam-remote-viewer-rtabmap` directory
and does not modify the existing ORB-SLAM3 environment. Set `SLAM_RTAB_WORK_ROOT`
to another dedicated temporary subdirectory if needed. `SLAM_BUILD_JOBS` accepts
1–32, defaulting to 4. Expect a substantial dependency download and several GB
of disk space; PCL/OpenCV pull transitive packages even though the RTAB-Map GUI
and physical camera SDKs are disabled in this build. Temporary files may be
removed by the operating system, so this is not a permanent application install.

The script checks the upstream commit, installs pinned primary dependency
versions in `env/`, builds RTAB-Map into `install/`, builds the sender in
`sender/`, runs native tests and checks `rtabmap_rgbd_sender --help`. Existing
source trees with changes or a different revision are rejected rather than
reset. Re-running can update dependency build variants within the pinned
versions; `conda-explicit.txt` records the exact resolved transitive packages.
This is not a bit-for-bit reproducibility guarantee.

The resulting executable is:

```text
/private/tmp/slam-remote-viewer-rtabmap/sender/rtabmap_rgbd_sender
```

To verify real image-pair input, use this executable with the dataset test below.
Bootstrap success itself does not prove camera capture, pose accuracy or Unity
rendering. No RGB-D device is accessed during installation.
This setup builds the RTAB-Map CLI; it does not add RTAB-Map selection to the
existing native macOS launcher UI.

Verification on the development Mac: a newly created dedicated dependency
environment, followed by a bootstrap rerun, completed the library/sender build,
all 13 native tests and executable startup. The resulting executable also
passed the full 794-pair dataset boundary test (794 poses, 111 point-cloud
messages). Six bootstrap preflight tests cover invalid arguments, unsafe roots,
missing conda, existing source preservation and dependency symlink protection:

```sh
python3 -m unittest discover -s tools -p test_bootstrap_rtabmap.py
```

Apple Silicon-specific checks are skipped on other hardware. First loading of
the real-library test took about 100 seconds on this host; the native test
timeout is 180 seconds.

### Manual / existing RTAB-Map installation

After installing RTAB-Map with GTSAM into an isolated prefix:

```sh
cmake -S sender/slam -B build/rtabmap \
  -DSLAM_ENABLE_RTABMAP=ON \
  -DCMAKE_PREFIX_PATH="/path/to/rtabmap;/path/to/dependencies"
cmake --build build/rtabmap
ctest --test-dir build/rtabmap --output-on-failure
```

For the Apple Silicon conda development environment, also set
`-DCMAKE_OSX_ARCHITECTURES=arm64`,
`-DCMAKE_EXE_LINKER_FLAGS=-L/path/to/dependencies/lib` and
`-DCMAKE_BUILD_RPATH=/path/to/dependencies/lib`. Without the latter, the linker
can succeed while executables fail to locate OpenCV or conda's C++ runtime.
The verified source-build configuration uses OpenCV 4.13.0, PCL 1.15.1, Eigen
3.4.0, GTSAM 4.2.0 and CMake 3.31.8. RTAB-Map itself additionally needed the
dependency prefix in `CMAKE_SHARED_LINKER_FLAGS`, and explicit SDK pthread
include/library paths on this host. The Apple Silicon bootstrap above includes
these settings; a manual build must supply equivalent dependency paths.

The current entry point is `rtabmap_rgbd_sender --help`. It accepts an association
file (`rgb_timestamp rgb_path depth_timestamp depth_path`), a Rust-owned Unix
socket, session/camera IDs, rectified intrinsics, depth units per metre, nominal
FPS, and a frame limit. Paths resolve relative to the association file.
Depth images are uint16; conversion to metres occurs before tracking. Pair
timestamps must differ by at most 20 ms and increase strictly. This entry point
processes image pairs offline; it is not yet a live RGB-D capture driver.

Start the existing Rust sender with `--source live --slam-socket SOCKET` before
starting this producer, then connect Receiver/Unity as for other live boundary
producers. Use a distinct socket, network endpoint and session per camera.
The camera metadata advertises `rgbd`; existing producers still default to
`monocular`. On input/backend/network failure the producer exits nonzero and
closes the connection without falsely reporting successful session completion.

## Real dataset boundary verification

```sh
cargo build --manifest-path sender/streamer/Cargo.toml --bin slam_boundary_dump
python3 tools/test-rtabmap-dataset.py /path/to/rgbd_dataset_freiburg1_xyz \
  --producer build/rtabmap/rtabmap_rgbd_sender \
  --validator sender/streamer/target/debug/slam_boundary_dump
python3 -m unittest discover -s tools -p test_rtabmap_dataset_test.py
```

The harness pairs RGB/depth frames within 20 ms without reusing depth frames,
then runs the actual producer against Rust's production boundary validator.
It retains logs and the association file in a reported temporary directory.
It requires clean completion, at least 90% tracked poses, and nonempty map
output. For TUM's pre-registered images it uses the recommended ROS default
pinhole intrinsics and depth factor 5000, without independently undistorting
the RGB image; see the [TUM format documentation](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats).
This approximation is specific to that dataset workflow, not a replacement for
calibrating a physical RGB-D camera.

The full `freiburg1_xyz` run on the development host completed with 794 paired
frames, 794 tracked poses, 111 point-cloud messages and 837,941 point operations,
with 65,982 observations in the final exported map.
The initial unsplit implementation failed on this same sequence at the 1 MiB
limit; the full run passed after bounded chunking. All 13 native tests and 3
association tests passed. This proves real-backend/boundary operation, not
trajectory accuracy, successful loop closure, or Unity rendering.
