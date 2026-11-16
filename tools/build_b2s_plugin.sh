#!/usr/bin/env bash
#
# Native build of the B2S Pro plugin (Linux, macOS, Windows/MSVC).
#
#   ./tools/build_b2s_plugin.sh              # Debug, build-dev/
#   ./tools/build_b2s_plugin.sh Release      # or: x86 (Windows, 32 bit toolset)
#
# Extra CMake options: B2S_PRO_CMAKE=' -DBUILD_SHARED_LIBS=ON' ./tools/build_b2s_plugin.sh
# Build tree location: CMAKE_BUILD_DIR=/tmp/b2s-build ./tools/build_b2s_plugin.sh

set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

CONFIG=Debug
EXTRA_CMAKE="${B2S_PRO_CMAKE:-}"
for a in "$@"; do
   case "$a" in
      Release|RelWithDebInfo|Debug|MinSizeRel) CONFIG="$a" ;;
      *) EXTRA_CMAKE="$EXTRA_CMAKE $a" ;;
   esac
done

# the plugin builds against the upstream tree, it is not hosted here
[ -d upstream ] || python3 fetch-upstream.py

BUILD_DIR="${CMAKE_BUILD_DIR:-build-dev}"
# shellcheck disable=SC2086
cmake -B "$BUILD_DIR" -S . -DCMAKE_BUILD_TYPE="$CONFIG" $EXTRA_CMAKE
cmake --build "$BUILD_DIR" --config "$CONFIG" --parallel
echo
echo "plugin library: $(find "$BUILD_DIR" -name 'plugin-b2spro*' \( -name '*.dll' -o -name '*.so' -o -name '*.dylib' \) -newer CMakeLists.txt -printf '%p (%s bytes)\n' 2>/dev/null | head -3)"
