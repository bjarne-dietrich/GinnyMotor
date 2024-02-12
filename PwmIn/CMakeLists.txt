add_library(PwmIn
    PwmIn.cpp
    PwmIn.h
    PwmIn.pio.h
)

pico_generate_pio_header(PwmIn ${CMAKE_CURRENT_LIST_DIR}/PwmIn.pio)

target_link_libraries(PwmIn
        pico_stdlib
        hardware_pio
        hardware_pwm
        hardware_gpio
        )

target_include_directories(PwmIn PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")