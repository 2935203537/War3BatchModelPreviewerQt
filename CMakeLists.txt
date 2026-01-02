cmake_minimum_required(VERSION 3.24)

project(War3BatchModelPreviewerQt
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Qt6 is recommended (MSVC 2022 kit). Set CMAKE_PREFIX_PATH to your Qt install, e.g.:
#   -DCMAKE_PREFIX_PATH="C:/Qt/6.6.2/msvc2019_64"
set(QT6_MIN_VERSION 6.2)
set(QT5_MIN_VERSION 5.15)

# MPQ support via StormLib
include(FetchContent)
set(STORMLIB_BUILD_SHARED OFF CACHE BOOL "" FORCE)
set(STORMLIB_BUILD_STATIC ON CACHE BOOL "" FORCE)
FetchContent_Declare(
  stormlib
  GIT_REPOSITORY https://github.com/ladislav-zezula/StormLib.git
  GIT_TAG        master
)
FetchContent_MakeAvailable(stormlib)
if (TARGET storm)
  target_compile_definitions(storm PRIVATE __STORMLIB_SELF__ __STORMLIB_NO_STATIC_LINK__)
endif()

# Tip for Visual Studio CMake: pass -DQT6_ROOT=... or set Qt6_DIR/CMAKE_PREFIX_PATH
if(DEFINED QT6_ROOT)
  list(PREPEND CMAKE_PREFIX_PATH "${QT6_ROOT}")
endif()

find_package(Qt6 ${QT6_MIN_VERSION} QUIET COMPONENTS Widgets OpenGLWidgets Concurrent)
if (NOT Qt6_FOUND)
  message(STATUS "Qt6 not found; trying Qt5...")
  find_package(Qt5 ${QT5_MIN_VERSION} REQUIRED COMPONENTS Widgets OpenGL Concurrent)
  set(USE_QT5 ON)
endif()

set(SOURCES
    src/main.cpp
    src/MainWindow.cpp
    src/MainWindow.h
    src/GLModelView.cpp
    src/GLModelView.h
    src/MdxLoader.cpp
    src/MdxLoader.h
    src/ModelData.h
    src/BlpLoader.cpp
    src/BlpLoader.h
    src/LogSink.cpp
    src/LogSink.h
    src/Vfs.cpp
    src/Vfs.h
)

if (USE_QT5)
  set(CMAKE_AUTOMOC ON)
  set(CMAKE_AUTOUIC ON)
  set(CMAKE_AUTORCC ON)
  add_executable(War3BatchModelPreviewerQt ${SOURCES})
else()
  qt_standard_project_setup()
  qt_add_executable(War3BatchModelPreviewerQt ${SOURCES})
endif()

target_compile_definitions(War3BatchModelPreviewerQt PRIVATE __STORMLIB_NO_STATIC_LINK__)

if (USE_QT5)
  target_link_libraries(War3BatchModelPreviewerQt PRIVATE Qt5::Widgets Qt5::OpenGL Qt5::Concurrent)
else()
  target_link_libraries(War3BatchModelPreviewerQt PRIVATE Qt6::Widgets Qt6::OpenGLWidgets Qt6::Concurrent)
endif()

if (TARGET storm)
  target_link_libraries(War3BatchModelPreviewerQt PRIVATE storm)
  target_include_directories(War3BatchModelPreviewerQt PRIVATE ${stormlib_SOURCE_DIR}/src)
elseif (TARGET StormLib::StormLib)
  target_link_libraries(War3BatchModelPreviewerQt PRIVATE StormLib::StormLib)
elseif (TARGET StormLib)
  target_link_libraries(War3BatchModelPreviewerQt PRIVATE StormLib)
endif()
if (MSVC)
    target_compile_options(War3BatchModelPreviewerQt PRIVATE /W4 /permissive-)
else()
    target_compile_options(War3BatchModelPreviewerQt PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Helpful for Windows: copy Qt runtime DLLs next to the exe when building from VS
if (WIN32 AND NOT USE_QT5)
    qt_generate_deploy_app_script(
        TARGET War3BatchModelPreviewerQt
        OUTPUT_SCRIPT deploy_script
        NO_UNSUPPORTED_PLATFORM_ERROR
    )
    install(SCRIPT ${deploy_script})
endif()
