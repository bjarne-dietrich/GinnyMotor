# What CMake to start at
cmake_minimum_required(VERSION 3.12)
 
# Include the subsidiary .cmake file to get the SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Initalise the SDK
pico_sdk_init()

# Set the name and version of the project
project(PicoTest VERSION 1.0.0)
 
# Link the Project to a source file (step 4.6)
add_executable(${PROJECT_NAME}
    main.c
)
 
# Link the Project to an extra library (pico_stdlib)
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
    hardware_pwm
)
 
# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# Enable USB, UART output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)