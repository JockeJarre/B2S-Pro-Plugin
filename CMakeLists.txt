# Standalone build of the official VPX B2S plugin (vpinball/vpinball 85914a85,
# VPinballX_BGFX 10.8.1 rev 6533). Mirrors make/CMakeLists_plugin_B2S.txt from
# that commit (same sources, include directories, MSVC options and output name),
# pointed at the unchanged copies under upstream/. src/ is the B2S Pro working
# copy of plugins/b2s; every change there is marked "B2S Pro:".

cmake_minimum_required(VERSION 3.25)
project(B2SPluginClean LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 99)

set(UP ${CMAKE_CURRENT_SOURCE_DIR}/upstream)

# The upstream files are not hosted in this repository: they are fetched, pinned by commit and
# SHA-256, from the official repositories (see fetch-upstream.py and the README 'Build from source')
if(NOT EXISTS "${UP}/third-party/include/tinyxml2/tinyxml2.h")
   message(FATAL_ERROR "The fetched upstream files are missing from upstream/. Run 'python3 fetch-upstream.py' (needs network access) and configure again.")
endif()

set(B2S_PLUGIN_SOURCES
   ${CMAKE_CURRENT_SOURCE_DIR}/src/common.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/common.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/B2SPlugin.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/B2SDataModel.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/B2SDataModel.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/B2SRenderer.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/B2SRenderer.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/B2SServer.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/B2SServer.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/pro/B2SProData.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/pro/B2SProData.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/pro/B2SProRuntime.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/pro/B2SProRuntime.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/pro/B2SProDynamicLight.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/pro/B2SProDynamicLight.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/pro/B2SProScore.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/pro/B2SProScore.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/pro/B2SProMotion.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/pro/B2SProMotion.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/pro/B2SProPhysics.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/pro/B2SProPhysics.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/DMDOverlay/DMDOverlay.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/DMDOverlay/DMDOverlay.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/DMDOverlay/OverlayDisplayOptions.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/seg/Brush.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/seg/Dream7Display.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/seg/Dream7Display.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/seg/GraphicsPath.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/seg/GraphicsPath.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/seg/LEDBox.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/seg/LEDBox.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/seg/Matrix.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/seg/Matrix.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/seg/SegDisplay.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/seg/SegGraphics.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/seg/SegGraphics.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/seg/SegTypes.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/seg/Segment.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/seg/Segment.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/seg/SegmentNumber.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/seg/SegmentNumber.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/seg/SegmentStyle.h

   ${UP}/plugins/nixie/NixieLayoutFile.h
   ${UP}/plugins/nixie/NixieLayoutFile.cpp
   ${UP}/plugins/plugins/ResURIResolver.h
   ${UP}/plugins/plugins/ResURIResolver.cpp

   ${UP}/third-party/include/tinyxml2/tinyxml2.h
   ${UP}/third-party/include/tinyxml2/tinyxml2.cpp
)

# Which sources a build came from: the git revision and the build time. Every build of this plugin looks identical
# on disk, and shipping 'the latest' blindly made us retest an already fixed build too many times: log it at load,
# stamp the binary's file properties with it and stamp the CI package name with it
find_package(Git QUIET)
set(B2S_BUILD_ID "local (no git)")
if(GIT_FOUND)
   execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse --short=9 HEAD OUTPUT_VARIABLE B2S_GIT_SHA OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
   if(B2S_GIT_SHA)
      string(TIMESTAMP B2S_BUILD_TIME "%Y-%m-%d %H:%M UTC" UTC)
      set(B2S_BUILD_ID "${B2S_GIT_SHA} (${B2S_BUILD_TIME})")
   endif()
endif()

if(WIN32 AND MSVC)
   # The Windows file properties (Explorer Details tab) of the library: name the file the build really is
   configure_file("${CMAKE_CURRENT_SOURCE_DIR}/src/version_info.rc.in" "${CMAKE_BINARY_DIR}/B2SPlugin_version_info.rc" @ONLY)
   list(APPEND B2S_PLUGIN_SOURCES "${CMAKE_BINARY_DIR}/B2SPlugin_version_info.rc")
endif()

add_library(B2SPlugin MODULE ${B2S_PLUGIN_SOURCES})

target_compile_definitions(B2SPlugin PRIVATE B2SPLUGIN_BUILD_ID="${B2S_BUILD_ID}")

target_include_directories(B2SPlugin PUBLIC
   ${UP}/third-party/include
   ${UP}/plugins
   ${CMAKE_CURRENT_SOURCE_DIR}/src
)

set_target_properties(B2SPlugin PROPERTIES
   MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
   PREFIX ""
   # The library names declared by plugin.cfg: plugin-b2spro64.dll on Windows (the flavor of the
   # official application), plugin-b2spro.so/.dylib elsewhere
   OUTPUT_NAME "$<$<PLATFORM_ID:Windows>:plugin-b2spro64>$<$<NOT:$<PLATFORM_ID:Windows>>:plugin-b2spro>"
)

# The optimizer flags of the official plugin build are MSVC spellings: apply them to MSVC only, so
# that a Release build with gcc/clang on the other platforms works
set(OPT_COMMON /Ob2 /O2 /Oi /fp:fast /fp:except- /Ot /GF /GS- /Gy /GR /Oy /GT /GL /w44005 /Zc:__cplusplus /std:c++20)
target_compile_options(B2SPlugin PUBLIC
   $<$<AND:$<CONFIG:Release>,$<COMPILE_LANGUAGE:CXX>,$<CXX_COMPILER_ID:MSVC>>:${OPT_COMMON}>
   $<$<AND:$<CONFIG:Release>,$<COMPILE_LANGUAGE:C>,$<C_COMPILER_ID:MSVC>>:${OPT_COMMON}>
)
target_link_options(B2SPlugin PUBLIC
   $<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:MSVC>>:/INCREMENTAL:NO>
   $<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:MSVC>>:/OPT:REF>
   $<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:MSVC>>:/OPT:ICF>
   $<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:MSVC>>:/LTCG>
)

add_custom_command(TARGET B2SPlugin POST_BUILD
   COMMAND "${CMAKE_COMMAND}" -E copy "${CMAKE_CURRENT_SOURCE_DIR}/src/plugin.cfg" "$<TARGET_FILE_DIR:B2SPlugin>/plugin.cfg"
)
