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
set(QT_MIN_VERSION 6.2)

# Tip for Visual Studio CMake: pass -DQT6_ROOT=... or set Qt6_DIR/CMAKE_PREFIX_PATH
if(DEFINED QT6_ROOT)
  list(PREPEND CMAKE_PREFIX_PATH "${QT6_ROOT}")
endif()

find_package(Qt6 ${QT_MIN_VERSION} REQUIRED COMPONENTS Widgets OpenGLWidgets Concurrent)

qt_standard_project_setup()

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
)

qt_add_executable(War3BatchModelPreviewerQt
    ${SOURCES}
)

target_link_libraries(War3BatchModelPreviewerQt
    PRIVATE Qt6::Widgets Qt6::OpenGLWidgets Qt6::Concurrent
)
if (MSVC)
    target_compile_options(War3BatchModelPreviewerQt PRIVATE /W4 /permissive-)
else()
    target_compile_options(War3BatchModelPreviewerQt PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Helpful for Windows: copy Qt runtime DLLs next to the exe when building from VS
if (WIN32)
    qt_generate_deploy_app_script(
        TARGET War3BatchModelPreviewerQt
        OUTPUT_SCRIPT deploy_script
        NO_UNSUPPORTED_PLATFORM_ERROR
    )
    install(SCRIPT ${deploy_script})
endif()
