# What CMake to start at
cmake_minimum_required(VERSION 3.12)
 
# Include the subsidiary .cmake file to get the SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
pico_sdk_init()

# Set the name
project(PicoTest
        VERSION 1.0.0
        LANGUAGES C CXX
        )
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


add_subdirectory(RP2040-motor-encoder/MotorEncoder)

# Link the Project to a source file (step 4.6)
add_executable(${PROJECT_NAME}
    main.cpp
)
 
# Link the Project to an extra library
target_link_libraries(${PROJECT_NAME} PUBLIC
    pico_stdlib
    hardware_pio
    hardware_pwm
    hardware_gpio
    pico_multicore
    MotorEncoder
)

target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_SOURCE_DIR}/RP2040-motor-encoder")

# Enable USB, UART output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 1)


# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})