cmake_minimum_required(VERSION 3.13)

# Locate the Pico SDK. Prefer an in-tree ./pico-sdk (handy for local work),
# otherwise fall back to the PICO_SDK_PATH environment variable, which is the
# standard Pico workflow (see README). The SDK is NOT committed to this repo.
if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/pico-sdk/pico_sdk_init.cmake)
    set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-sdk)
endif()

# Target the Raspberry Pi Pico (RP2040) by default.
set(PICO_BOARD pico CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(usbrx C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

# pico_spdif_rx's CMake links an unused 'pico_audio_32b' (only needed by its I2S
# sample, not by the receiver itself). Provide an empty stub so we can reuse the
# library's CMake without pulling in pico-extras.
if (NOT TARGET pico_audio_32b)
    add_library(pico_audio_32b INTERFACE)
endif()

add_subdirectory(lib/pico_spdif_rx pico_spdif_rx)

add_executable(usbrx
    src/main.c
    src/usb_descriptors.c
)

# Make tusb_config.h (in src/) visible to the TinyUSB sources, which are compiled
# in this executable's context by the SDK.
target_include_directories(usbrx PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(usbrx
    pico_stdlib
    pico_spdif_rx
    tinyusb_device
)

# Assign PIO0 / DMA_IRQ0 to the S/PDIF receiver (its defaults; stated explicitly).
target_compile_definitions(usbrx PRIVATE
    PICO_SPDIF_RX_PIO=0
    PICO_SPDIF_RX_DMA_IRQ=0
)

# Debug log over UART0 (GP0/GP1); keep USB free for audio.
pico_enable_stdio_uart(usbrx 1)
pico_enable_stdio_usb(usbrx 0)

# Produce .uf2 / .elf / .bin / .map etc.
pico_add_extra_outputs(usbrx)
