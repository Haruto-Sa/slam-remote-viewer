# RTAB-Map RGB-D backend (work in progress)

Tracked in [#108](https://github.com/Haruto-Sa/slam-remote-viewer/issues/108).
This backend is not yet release-ready. Real dataset and end-to-end viewer
verification, live camera capture, installation automation and independent
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
  the map. The existing 1 MiB boundary-message limit also applies. Export is
  approximately once per second of input frames, plus a final export at EOF.

## Optional build and dataset entry point

After installing RTAB-Map with GTSAM into an isolated prefix:

```sh
cmake -S sender/slam -B build/rtabmap \
  -DSLAM_ENABLE_RTABMAP=ON \
  -DCMAKE_PREFIX_PATH="/path/to/rtabmap;/path/to/dependencies"
cmake --build build/rtabmap
ctest --test-dir build/rtabmap --output-on-failure
```

For the Apple Silicon conda development environment, also set
`-DCMAKE_EXE_LINKER_FLAGS=-L/path/to/dependencies/lib` and
`-DCMAKE_BUILD_RPATH=/path/to/dependencies/lib`. Without the latter, the linker
can succeed while executables fail to locate OpenCV or conda's C++ runtime.
The verified source-build configuration uses OpenCV 4.13.0, PCL 1.15.1, Eigen
3.4.0, GTSAM 4.2.0 and CMake 3.31.8. RTAB-Map itself additionally needed the
dependency prefix in `CMAKE_SHARED_LINKER_FLAGS`, and explicit SDK pthread
include/library paths on this host. Reproducible bootstrap automation is still
pending; this is not yet a one-command installation guide.

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
