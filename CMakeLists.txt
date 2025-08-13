cmake_minimum_required(VERSION 3.10)
project(nova_engine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED 
    gstreamer-1.0
    gstreamer-app-1.0
    gstreamer-video-1.0
    gstreamer-base-1.0
)

# Find ISA-L
find_path(ISAL_INCLUDE_DIR isa-l/erasure_code.h)
find_library(ISAL_LIBRARY isal)

# Find CUDA
find_package(CUDA)
if(CUDA_FOUND)
    add_definitions(-DHAS_CUDA)
endif()

# Find VAAPI
pkg_check_modules(VAAPI libva)
if(VAAPI_FOUND)
    add_definitions(-DHAS_VAAPI)
endif()

# Include directories
include_directories(
    ${GSTREAMER_INCLUDE_DIRS}
    ${ISAL_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIRS}
    ${VAAPI_INCLUDE_DIRS}
)

# Link directories
link_directories(
    ${GSTREAMER_LIBRARY_DIRS}
    ${VAAPI_LIBRARY_DIRS}
)

# Add source files
add_executable(nova_engine
    src/nova_engine.cpp
    src/gpu_detect.cpp
)

# Link libraries
target_link_libraries(nova_engine
    ${GSTREAMER_LIBRARIES}
    ${ISAL_LIBRARY}
    ${CUDA_LIBRARIES}
    ${VAAPI_LIBRARIES}
    pthread
)