cmake_minimum_required(VERSION 3.15)
project(smartInset VERSION 1.0.0 LANGUAGES CXX)

include(FetchContent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#
# ---- Platform specific settings ----
#

if (WIN32)
    message(STATUS "Configuring for Windows")
    if (NOT DEFINED LXSDK_PATH)
        set(LXSDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../LXSDK-000025")
    endif()
    add_compile_options(/MP /W3)

elseif (APPLE)
    message(STATUS "Configuring for macOS")
    if (NOT DEFINED LXSDK_PATH)
        set(LXSDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../LXSDK-000025")
    endif()
    set(CMAKE_OSX_ARCHITECTURES "x86_64;arm64" CACHE STRING "macOS architectures" FORCE)

else()
    message(STATUS "Configuring for Linux / Other")
    if (NOT DEFINED LXSDK_PATH)
        set(LXSDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../LXSDK-000025")
    endif()
endif()

# Allow environment variable override
if (DEFINED ENV{LXSDK_PATH})
    set(LXSDK_PATH "$ENV{LXSDK_PATH}")
endif()

#
# ---- LXSDK ----
#
if (NOT EXISTS "${LXSDK_PATH}/include")
    message(STATUS "LXSDK not found at '${LXSDK_PATH}'. Fetching via FetchContent...")
    FetchContent_Declare(
        lxsdk
        URL https://thefoundry.s3.us-east-1.amazonaws.com/products/modo/16.1v5/lxsdk16.1v5_692025.zip
    )
    FetchContent_Populate(lxsdk)
    message(STATUS "Fetched lxsdk directory: ${lxsdk_SOURCE_DIR}")
    set(LXSDK_PATH ${lxsdk_SOURCE_DIR})
endif()

if (NOT EXISTS "${LXSDK_PATH}/include")
    message(FATAL_ERROR "LXSDK not found at: ${LXSDK_PATH}")
endif()

file(GLOB LXSDK_SOURCES "${LXSDK_PATH}/common/*.cpp")
message(STATUS "LXSDK Path: ${LXSDK_PATH}")

add_library(lxsdk STATIC ${LXSDK_SOURCES})
target_include_directories(lxsdk PUBLIC "${LXSDK_PATH}/include")
target_compile_definitions(lxsdk PRIVATE GL_SILENCE_DEPRECATION)
set_target_properties(lxsdk PROPERTIES POSITION_INDEPENDENT_CODE ON)

#
# ---- SmartInset Plugin ----
#

add_library(smartInset SHARED
    source/initialize.cpp
    source/inset_command.cpp
    source/inset_tool.cpp
    source/safe_inset.cpp
    source/padding_inset.cpp
)

target_include_directories(smartInset PRIVATE
    source
    "${LXSDK_PATH}/include"
)

target_compile_definitions(smartInset PRIVATE
    GL_SILENCE_DEPRECATION
)

target_link_libraries(smartInset PRIVATE
    lxsdk
)

#
# ---- Tests ----
#
enable_testing()
add_executable(test_inset
    test/test_inset_math.cpp
    source/safe_inset.cpp
    source/padding_inset.cpp
)
target_include_directories(test_inset PRIVATE
    source
    "${LXSDK_PATH}/include"
)
target_compile_definitions(test_inset PRIVATE
    INSET_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test"
)
add_test(NAME InsetMathTest COMMAND test_inset)

#
# ---- Install (Kit layout) ----
#

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/SmartInsetKit" CACHE PATH "Install prefix" FORCE)
endif()

install(FILES "index.cfg" DESTINATION "${CMAKE_INSTALL_PREFIX}")
install(FILES "README.html" DESTINATION "${CMAKE_INSTALL_PREFIX}")
install(FILES "SPEC.md" DESTINATION "${CMAKE_INSTALL_PREFIX}")
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Icons")
    install(DIRECTORY "Icons" DESTINATION "${CMAKE_INSTALL_PREFIX}")
endif()
install(TARGETS smartInset DESTINATION "${CMAKE_INSTALL_PREFIX}/extra")
