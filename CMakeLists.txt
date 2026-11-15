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

add_library(B2SPlugin MODULE ${B2S_PLUGIN_SOURCES})

target_include_directories(B2SPlugin PUBLIC
   ${UP}/third-party/include
   ${UP}/plugins
   ${CMAKE_CURRENT_SOURCE_DIR}/src
)

set_target_properties(B2SPlugin PROPERTIES
   MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
   PREFIX ""
   OUTPUT_NAME "plugin-b2spro64"
)

set(OPT_COMMON /Ob2 /O2 /Oi /fp:fast /fp:except- /Ot /GF /GS- /Gy /GR /Oy /GT /GL /w44005 /Zc:__cplusplus /std:c++20)
target_compile_options(B2SPlugin PUBLIC
   $<$<CONFIG:Release>:$<$<COMPILE_LANGUAGE:CXX>:${OPT_COMMON}>>
   $<$<CONFIG:Release>:$<$<COMPILE_LANGUAGE:C>:${OPT_COMMON}>>
)
target_link_options(B2SPlugin PUBLIC
   $<$<CONFIG:Release>:/INCREMENTAL:NO>
   $<$<CONFIG:Release>:/OPT:REF>
   $<$<CONFIG:Release>:/OPT:ICF>
   $<$<CONFIG:Release>:/LTCG>
)

add_custom_command(TARGET B2SPlugin POST_BUILD
   COMMAND "${CMAKE_COMMAND}" -E copy "${CMAKE_CURRENT_SOURCE_DIR}/src/plugin.cfg" "$<TARGET_FILE_DIR:B2SPlugin>/plugin.cfg"
)
