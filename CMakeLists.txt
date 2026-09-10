cmake_minimum_required(VERSION 3.16)

if (ESP_PLATFORM)
    # ESP32 BUILD
    idf_component_register(
        INCLUDE_DIRS "include"
    )

    target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++20)

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