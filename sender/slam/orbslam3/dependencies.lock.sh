# Revisions used by tools/bootstrap-orbslam3-macos.sh.
# Upstream source trees and build products stay outside this repository.
ORB_SLAM3_REPOSITORY="https://github.com/UZ-SLAMLab/ORB_SLAM3.git"
ORB_SLAM3_REVISION="4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4"
PANGOLIN_REPOSITORY="https://github.com/stevenlovegrove/Pangolin.git"
PANGOLIN_REVISION="aff6883c83f3fd7e8268a9715e84266c42e2efe3"

# Defaults-channel osx-arm64 packages. Keep the versions explicit so that the
# resulting native dependency graph can be reproduced without Intel Homebrew.
CONDA_CMAKE_VERSION="3.31.2"
CONDA_EIGEN_VERSION="3.4.0"
CONDA_OPENCV_VERSION="4.10.0"
CONDA_BOOST_VERSION="1.82.0"
CONDA_OPENSSL_VERSION="3.5.7"
CONDA_GLEW_VERSION="2.2.0"
