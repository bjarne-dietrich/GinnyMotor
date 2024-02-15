add_library(MotorEncoder
    MotorEncoder.cpp
    MotorEncoder.h
    MotorEncoder.pio.h
)

pico_generate_pio_header(MotorEncoder ${CMAKE_CURRENT_LIST_DIR}/MotorEncoder.pio)

target_link_libraries(MotorEncoder
        pico_stdlib
        hardware_pio
        )

target_include_directories(MotorEncoder PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")