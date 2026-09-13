cmake_minimum_required(VERSION 3.16)

if (ESP_PLATFORM)
    # ESP32 BUILD
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)

    project(GNC-LIB)

else()
    # PC BUILD
    project(libgnc_host_tests CXX)

    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

    # STATIC or SHARED if has src/ else use INTERFACE
    add_library(libgnc INTERFACE)
    target_include_directories(libgnc INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

    enable_testing()
    add_subdirectory(tests)
endif()