#!/bin/bash
# Usage: ./scripts/build_all_static.sh /PATH/TO/spectacularAI_*_static.tar.gz

set -eux

SDK_PACKAGE=$1

ROOT=$(pwd)
TMP="$ROOT/tmp"

# Install the static library SDK variant into a temporary location
mkdir -p "$TMP/unpack"
cp "$SDK_PACKAGE" "$TMP/unpack/"
cd "$TMP/unpack"
tar xvf spectacularAI_*.tar.gz
make PREFIX="$TMP/install" test
make install

cd "$ROOT"
./scripts/build_all.sh -DspectacularAI_DIR="$TMP/install/lib/cmake/spectacularAI"

INSTALL_DIR="spectacularai_ros2/install/spectacularai_ros2/"
# Strip the executable (just in case the CMake build failed to do that)
strip "$INSTALL_DIR/lib/libspectacularai_ros2.so"

# Copy license/notice file form the SDK package
LICENSE_OUTPUT_DIR="$INSTALL_DIR"
mkdir -p "$LICENSE_OUTPUT_DIR"
cp $TMP/install/share/doc/spectacularAI/LICENSE "$LICENSE_OUTPUT_DIR/LICENSE.txt"

# Delete the temporary static library installation and build dir
rm -rf "$TMP"
rm -rf "spectacularai_ros2/build"
