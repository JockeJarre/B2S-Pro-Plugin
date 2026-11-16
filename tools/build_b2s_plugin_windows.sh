#!/usr/bin/env bash
#
# Cross-compile the B2S Pro plugin Windows DLL from Linux or macOS with mingw-w64.
#
#   ./tools/build_b2s_plugin_windows.sh            # x64, Release
#   ./tools/build_b2s_plugin_windows.sh x64 Debug
#   ./tools/build_b2s_plugin_windows.sh x86        # 32 bit plugin-b2spro.dll
#
# Output: build-b2s-win-<arch>/b2spro/ (plugin DLL + plugin.cfg + the mingw runtime DLLs,
# ready to be dropped in the Visual Pinball 'plugins/b2spro' folder).
#
# The MSVC build is the one the GitHub Actions workflow runs; this script is for quick
# testing from a POSIX box. Same sources, the version resource is compiled by windres.
#
# Dependencies: cmake, ninja or make, python3 (fetch-upstream.py) and mingw-w64:
#   Debian/Ubuntu:  sudo apt install mingw-w64
#   MSYS2 UCRT64:   pacman -S mingw-w64-ucrt-x86_64-gcc
#
# No root on a Debian/Ubuntu box: the toolchain can be extracted in the home directory, the
# gcc/g++ binaries find their install directory relative to themselves so a plain 'dpkg -x'
# tree works (the -posix variants are used, the win32 ones have no POSIX threads):
#   mkdir -p ~/.local/mingw/dl ~/.local/mingw/root; cd ~/.local/mingw/dl
#   apt-get download binutils-mingw-w64-base binutils-mingw-w64-x86-64 mingw-w64-common \
#      mingw-w64-x86-64-dev gcc-mingw-w64-base gcc-mingw-w64-x86-64-posix-runtime \
#      gcc-mingw-w64-x86-64-posix g++-mingw-w64-x86-64-posix
#   for d in *.deb; do dpkg -x "$d" ~/.local/mingw/root; done
#   cd ~/.local/mingw/root/usr/bin
#   ln -sf x86_64-w64-mingw32-gcc-posix  x86_64-w64-mingw32-gcc
#   ln -sf x86_64-w64-mingw32-g++-posix  x86_64-w64-mingw32-g++
#   ln -sf i686-w64-mingw32-gcc-posix    i686-w64-mingw32-gcc
#   ln -sf i686-w64-mingw32-g++-posix    i686-w64-mingw32-g++
#   export PATH=~/.local/mingw/root/usr/bin:$PATH

set -euo pipefail

ARCH="${1:-x64}"
CONFIG="${2:-Release}"

cd "$(dirname "${BASH_SOURCE[0]}")/.."

case "${ARCH}" in
   x64)
      CC=x86_64-w64-mingw32-gcc
      CXX=x86_64-w64-mingw32-g++
      SYSTEM_PROCESSOR=AMD64
      pacman_pkg=mingw-w64-ucrt-x86_64-gcc
      ;;
   x86)
      CC=i686-w64-mingw32-gcc
      CXX=i686-w64-mingw32-g++
      SYSTEM_PROCESSOR=x86
      pacman_pkg=mingw-w64-i686-gcc
      ;;
   *)
      echo "usage: $0 [x64|x86] [Release|Debug]" >&2
      exit 1
      ;;
esac

for tool in cmake python3 "${CC}" "${CXX}"; do
   if ! command -v "$tool" >/dev/null; then
      cat >&2 <<EOF
$tool not found; install the build dependencies:
  Debian/Ubuntu: sudo apt install cmake python3 mingw-w64
  MSYS2 UCRT64:  pacman -S ${pacman_pkg} mingw-w64-ucrt-x86_64-cmake
EOF
      exit 1
   fi
done

# the plugin builds against the upstream tree, it is not hosted here
[ -d upstream ] || python3 fetch-upstream.py

BUILD_DIR="build-b2s-win-${ARCH}"

cmake \
   -DCMAKE_SYSTEM_NAME=Windows \
   -DCMAKE_SYSTEM_PROCESSOR="${SYSTEM_PROCESSOR}" \
   -DCMAKE_C_COMPILER="${CC}" \
   -DCMAKE_CXX_COMPILER="${CXX}" \
   -DCMAKE_BUILD_TYPE="${CONFIG}" \
   -B "${BUILD_DIR}"

cmake --build "${BUILD_DIR}" --config "${CONFIG}" --parallel "$(nproc)"

# stage the plugin folder: the built DLL (CMake puts it under <build>/<config>/, mingw
# omits the config subdirectory for single-config generators when everything is Release)
OUT="${BUILD_DIR}/b2spro"
rm -rf "${OUT}"
mkdir -p "${OUT}"
LIB_NAME=$([ "${ARCH}" = x64 ] && echo plugin-b2spro64.dll || echo plugin-b2spro.dll)
lib="$(find "${BUILD_DIR}" -name "${LIB_NAME}" -print -quit)"
if [ -z "${lib}" ]; then
   echo "error: ${LIB_NAME} not found under ${BUILD_DIR}" >&2
   exit 1
fi
cp "${lib}" "${OUT}/"
cp src/plugin.cfg "${OUT}/"

# the mingw runtime next to the plugin so the folder can be dropped into a Visual Pinball
# installation as is ('-print-file-name' resolves them inside the toolchain)
case "${ARCH}" in
   x64) gcc_runtime=libgcc_s_seh-1.dll ;;
   *)   gcc_runtime=libgcc_s_dw2-1.dll ;;
esac
for dll in libstdc++-6.dll "${gcc_runtime}" libwinpthread-1.dll; do
   src="$("${CXX}" -print-file-name="${dll}")"
   if [ -f "${src}" ]; then
      cp -f "${src}" "${OUT}/"
   else
      echo "warning: ${dll} not found in the toolchain, the plugin will need it from an official VPX build" >&2
   fi
done

echo
echo "Built: ${OUT}/${LIB_NAME} (${CONFIG})"
ls -la "${OUT}"
