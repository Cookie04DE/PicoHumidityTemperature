cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(pico_temperature_humidity)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project

add_subdirectory(dht)

add_executable(pico_temperature_humidity
    pico_temperature_humidity.c
)

pico_enable_stdio_usb(pico_temperature_humidity 1)
pico_enable_stdio_uart(pico_temperature_humidity 0)

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()

target_include_directories(pico_temperature_humidity PRIVATE ${CMAKE_CURRENT_LIST_DIR} )

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(pico_temperature_humidity pico_stdlib dht pico_cyw43_arch_lwip_threadsafe_background hardware_rtc)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pico_temperature_humidity)
