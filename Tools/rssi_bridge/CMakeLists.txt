cmake_minimum_required(VERSION 3.10)
project(rssi_bridge CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(RSSI_BRIDGE_FETCH_NLOHMANN_JSON "Fetch nlohmann/json if not available" ON)
option(RSSI_BRIDGE_FETCH_MAVLINK "Fetch MAVLink C headers if not available" ON)

# Prefer ArduPilot-generated MAVLink headers if present.
# You can override with -DMAVLINK_INCLUDE_DIR=/path/to/mavlink/include
set(ARDUPILOT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(MAVLINK_INCLUDE_DIR "" CACHE PATH "Path containing mavlink/v2.0/<dialect>/mavlink.h")

if(MAVLINK_INCLUDE_DIR STREQUAL "")
    set(_candidate_dirs
        "${ARDUPILOT_ROOT}/build/sitl/libraries/GCS_MAVLink/include"
        "${ARDUPILOT_ROOT}/build/CubeBlack/libraries/GCS_MAVLink/include"
    )
    foreach(_dir IN LISTS _candidate_dirs)
        if(EXISTS "${_dir}/mavlink/v2.0/common/mavlink.h")
            set(MAVLINK_INCLUDE_DIR "${_dir}")
            break()
        endif()
    endforeach()
endif()

if(RSSI_BRIDGE_FETCH_NLOHMANN_JSON)
    include(FetchContent)
    if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
        FetchContent_Declare(
            nlohmann_json
            URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
            DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        )
    else()
        FetchContent_Declare(
            nlohmann_json
            URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
        )
    endif()
    FetchContent_MakeAvailable(nlohmann_json)
endif()

set(RSSI_BRIDGE_SOURCES
    main.cpp
    mavlink_sender.cpp
    rssi_processor.cpp
)

if(WIN32)
    list(APPEND RSSI_BRIDGE_SOURCES
        http_client_win.cpp
        serial_port_win.cpp
    )
else()
    find_package(CURL REQUIRED)
    list(APPEND RSSI_BRIDGE_SOURCES
        http_client.cpp
        serial_port.cpp
    )
endif()

add_executable(rssi_bridge
    ${RSSI_BRIDGE_SOURCES}
)

set(_mavlink_flat_layout OFF)

if(MAVLINK_INCLUDE_DIR STREQUAL "" OR NOT EXISTS "${MAVLINK_INCLUDE_DIR}/mavlink/v2.0/ardupilotmega/mavlink.h")
    if(RSSI_BRIDGE_FETCH_MAVLINK)
        include(FetchContent)
        FetchContent_Declare(
            mavlink_c_library_v2
            GIT_REPOSITORY https://github.com/mavlink/c_library_v2.git
            GIT_TAG master
        )
        FetchContent_MakeAvailable(mavlink_c_library_v2)
        set(MAVLINK_INCLUDE_DIR "${mavlink_c_library_v2_SOURCE_DIR}")
        set(_mavlink_flat_layout ON)
    else()
        message(FATAL_ERROR "Could not find MAVLink headers. Build ArduPilot once (SITL is fine) to generate them, pass -DMAVLINK_INCLUDE_DIR=..., or enable RSSI_BRIDGE_FETCH_MAVLINK.")
    endif()
endif()

target_include_directories(rssi_bridge PRIVATE "${MAVLINK_INCLUDE_DIR}")

if(_mavlink_flat_layout)
    target_compile_definitions(rssi_bridge PRIVATE RSSI_BRIDGE_MAVLINK_FLAT_LAYOUT=1)
endif()

if(WIN32)
    target_link_libraries(rssi_bridge PRIVATE winhttp)
else()
    target_link_libraries(rssi_bridge PRIVATE CURL::libcurl)
endif()

if(TARGET nlohmann_json::nlohmann_json)
    target_link_libraries(rssi_bridge PRIVATE nlohmann_json::nlohmann_json)
endif()
