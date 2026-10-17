#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin
#
# Builds OpenCV 5 from its official source and installs it, with only the
# modules calib uses, for Linux, whose packages are OpenCV 4 (README,
# "Prerequisites"). CI's opencv action runs it, as a developer does:
#
#   tools/install-opencv.sh VERSION PREFIX   # e.g. latest /usr/local
#   tools/install-opencv.sh --latest         # print the newest 5.x release
#
# VERSION is a release tag (5.0.0) or `latest`, the newest 5.x release, so
# Linux follows OpenCV's releases as Homebrew does on macOS. The modules are
# calib's (core, imgproc, objdetect, geometry) and its tests' (imgcodecs,
# calib), with what they depend on, and JPEG and PNG from OpenCV's bundled
# sources, so the install needs nothing beyond the C and C++ runtimes. It
# downloads nothing at configure time and builds no Python, Java, apps, tests,
# examples or docs. The build needs git, CMake and a C++ compiler; if PREFIX is
# not writable, the install step runs under sudo.
set -euo pipefail

repo=https://github.com/opencv/opencv.git

usage() {
  sed -n 's/^#   //p' "$0" >&2
  exit 2
}

# The newest tag of the form 5.X.Y: pre-releases (5.0.0-alpha) are skipped.
latest() {
  git ls-remote --tags --refs "$repo" 'refs/tags/5.*' |
    sed 's#.*refs/tags/##' |
    grep -E '^5\.[0-9]+\.[0-9]+$' |
    sort -V |
    tail -n 1
}

if [ "${1:-}" = --latest ]; then
  [ $# -eq 1 ] || usage
  version=$(latest)
  [ -n "$version" ] || {
    echo "no OpenCV 5 release tag at $repo" >&2
    exit 1
  }
  echo "$version"
  exit 0
fi
[ $# -eq 2 ] || usage
version=$1
prefix=$2
if [ "$version" = latest ]; then
  version=$(latest)
fi
case "$version" in
  5.*) ;;
  *)
    echo "VERSION must be an OpenCV 5 release tag or latest, not '$version'" >&2
    exit 2
    ;;
esac

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
jobs=${CMAKE_BUILD_PARALLEL_LEVEL:-$(getconf _NPROCESSORS_ONLN)}

git -c advice.detachedHead=false clone --quiet --depth 1 --branch "$version" \
  "$repo" "$work/src"
cmake -S "$work/src" -B "$work/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$prefix" \
  -DBUILD_LIST=core,imgproc,objdetect,geometry,imgcodecs,calib \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF \
  -DBUILD_DOCS=OFF -DBUILD_opencv_apps=OFF -DBUILD_JAVA=OFF \
  -DBUILD_opencv_python3=OFF -DOPENCV_PYTHON_SKIP_DETECTION=ON \
  -DOPENCV_GENERATE_SETUPVARS=OFF \
  -DWITH_JPEG=ON -DBUILD_JPEG=ON -DWITH_PNG=ON -DBUILD_PNG=ON \
  -DBUILD_ZLIB=ON \
  -DWITH_AVIF=OFF -DWITH_JASPER=OFF -DWITH_OPENJPEG=OFF -DWITH_OPENEXR=OFF \
  -DWITH_TIFF=OFF -DWITH_WEBP=OFF -DWITH_IMGCODEC_GIF=OFF -DWITH_UNIFONT=OFF \
  -DWITH_IPP=OFF -DWITH_ITT=OFF -DWITH_KLEIDICV=OFF -DWITH_LAPACK=OFF \
  -DWITH_EIGEN=OFF -DWITH_OPENCL=OFF -DWITH_VA=OFF -DWITH_VA_INTEL=OFF \
  -DWITH_PROTOBUF=OFF -DWITH_FLATBUFFERS=OFF -DWITH_VTK=OFF \
  -DWITH_FFMPEG=OFF -DWITH_GSTREAMER=OFF -DWITH_GTK=OFF -DWITH_V4L=OFF \
  -DWITH_OBSENSOR=OFF
cmake --build "$work/build" --parallel "$jobs"
if mkdir -p "$prefix" 2>/dev/null && [ -w "$prefix" ]; then
  cmake --install "$work/build"
else
  sudo cmake --install "$work/build"
fi
echo "OpenCV $version installed in $prefix"
