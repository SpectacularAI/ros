#!/bin/bash
# Usage: ./scripts/build_all_static.sh /PATH/TO/spectacularAI_*_static.tar.gz

set -eux

SDK_PACKAGE=$1

ROOT=$(pwd)
TMP="$ROOT/tmp"

mkdir -p "$TMP/unpack"
cp "$SDK_PACKAGE" "$TMP/unpack/"
cd "$TMP/unpack"
tar xvf spectacularAI_*.tar.gz
make PREFIX="$TMP/install" test

cd "$ROOT"
./scripts/build_all.sh -DspectacularAI_DIR="$TMP/install/lib/cmake/spectacularAI"
rm -rf "$TMP"
