
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(Tarefa_Revisao C CXX ASM)

pico_sdk_init()

add_executable(${PROJECT_NAME}
    Tarefa_Revisao.c
    inc/ssd1306.c
    )

pico_set_program_name(${PROJECT_NAME} "Tarefa_Revisao")
pico_set_program_version(${PROJECT_NAME} "0.1")

pico_generate_pio_header(${PROJECT_NAME} 
    ${CMAKE_CURRENT_LIST_DIR}/ws2818b.pio
    )

pico_enable_stdio_uart(${PROJECT_NAME} 1)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        hardware_pwm
        hardware_adc
        hardware_pio
        hardware_i2c
        hardware_timer
        pico_multicore)

target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/inc
)

pico_add_extra_outputs(${PROJECT_NAME})

