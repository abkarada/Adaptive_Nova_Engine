cmake_minimum_required(VERSION 3.16)
project(media_engine_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

# GStreamer
pkg_check_modules(GST REQUIRED
        gstreamer-1.0
        gstreamer-video-1.0
        gstreamer-rtp-1.0
        gstreamer-pbutils-1.0
)

# ⇩⇩⇩ YENİ: GLib/GObject/GIO (g_socket_* bunlarda)
pkg_check_modules(GLIB2 REQUIRED
        glib-2.0
        gobject-2.0
        gio-2.0
)

add_executable(nova_engine
        src/nova_engine.cpp
        src/gpu_detect.cpp
)

target_include_directories(nova_engine PRIVATE
        ${GST_INCLUDE_DIRS}
        ${GLIB2_INCLUDE_DIRS}     # ⇦ eklendi
)

target_link_directories(nova_engine PRIVATE
        ${GST_LIBRARY_DIRS}
        ${GLIB2_LIBRARY_DIRS}     # ⇦ eklendi (çoğu zaman gerekmez ama zararı yok)
)

target_compile_options(nova_engine PRIVATE
        ${GST_CFLAGS_OTHER}
        ${GLIB2_CFLAGS_OTHER}     # ⇦ eklendi
)

target_link_libraries(nova_engine PRIVATE
        ${GST_LIBRARIES}
        ${GLIB2_LIBRARIES}        # ⇦ en kritik satır (gio-2.0, gobject-2.0, glib-2.0)
        Threads::Threads
)
