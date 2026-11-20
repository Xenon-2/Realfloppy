# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(realfloppy C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()
add_subdirectory(no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/src build)
add_executable(realfloppy
    main.cpp
    usb_descriptors.c
    rf_protocol.cpp
    hw_config.cpp
    floppystation.cpp
)

target_include_directories(realfloppy PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(realfloppy
    pico_stdlib
    hardware_clocks
    hardware_gpio
    hardware_sync
    hardware_uart
    tinyusb_device
    tinyusb_board
    hardware_spi
    hardware_i2c
    pico_i2c_slave
    no-OS-FatFS-SD-SDIO-SPI-RPi-Pico            
)
target_compile_definitions(realfloppy PRIVATE
    PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=1
)
pico_enable_stdio_usb(realfloppy 1)
pico_enable_stdio_uart(realfloppy 0)

pico_add_extra_outputs(realfloppy)

