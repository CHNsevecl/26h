cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH /Users/macse/.pico-sdk/sdk/2.3.0 CACHE PATH "Path to Pico SDK")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_TYPE Debug)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# 1. 引入 Pico SDK
include(pico_sdk_import.cmake)

# 2. 声明项目
project(freeRTOS C CXX ASM)

# 3. 初始化 Pico SDK
pico_sdk_init()

set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2350_ARM_NTZ/FreeRTOS_Kernel_import.cmake)

# Add executable
file(GLOB_RECURSE SOURCES "src/*.cpp" "src/*.c")
add_executable(freeRTOS main.cpp ${SOURCES})

pico_set_program_name(freeRTOS "freeRTOS")
pico_set_program_version(freeRTOS "0.1")

pico_enable_stdio_uart(freeRTOS 0)
pico_enable_stdio_usb(freeRTOS 1)

target_link_libraries(freeRTOS
    pico_stdlib
    pico_multicore
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    hardware_i2c
)

target_include_directories(freeRTOS PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/include
)

pico_add_extra_outputs(freeRTOS)