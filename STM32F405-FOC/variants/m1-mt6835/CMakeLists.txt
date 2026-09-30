cmake_minimum_required(VERSION 3.22)
project(405_FOC C ASM)
option(FOC_DUAL "Run M0 and M1 together with TLE5012B encoders" OFF)
set(FOC_PORT "M0" CACHE STRING "Three-phase power connector: M0 or M1")
set(FOC_ENCODER "TLE5012B" CACHE STRING "Angle sensor model")
set(FOC_MOTOR "ZH3620_1" CACHE STRING "Motor model (reference model name not yet known)")
set(FOC_LOAD_PROFILE "PLASTIC_ARM" CACHE STRING "Mechanical load profile: FREE_SHAFT or PLASTIC_ARM")
set(FOC_INSTALLATION_ID "1" CACHE STRING "Physical sensor/magnet/phase installation: 1..15")
set(FOC_M1_INSTALLATION_ID "2" CACHE STRING "M1 physical installation: 1..15")
set(FOC_PROTECTION "WARN" CACHE STRING "Threshold policy: WARN or TRIP")
set(FOC_USB_DIVIDER "2" CACHE STRING "USB frame every N current samples")
set(FOC_M0_ADC_MODE "SEQUENTIAL" CACHE STRING "M0 current acquisition: SEQUENTIAL or DUAL")
set_property(CACHE FOC_M0_ADC_MODE PROPERTY STRINGS SEQUENTIAL DUAL)
if(FOC_M0_ADC_MODE STREQUAL "DUAL")
    set(FOC_M0_DUAL_ADC 1)
elseif(FOC_M0_ADC_MODE STREQUAL "SEQUENTIAL")
    set(FOC_M0_DUAL_ADC 0)
else()
    message(FATAL_ERROR "FOC_M0_ADC_MODE must be SEQUENTIAL or DUAL")
endif()
set(FOC_M0_TRIGGER_TICKS "" CACHE STRING "Optional M0 ADC trigger override for sampling experiments")
if(NOT FOC_M0_TRIGGER_TICKS STREQUAL "")
    if(NOT FOC_M0_TRIGGER_TICKS MATCHES "^[0-9]+$" OR
       FOC_M0_TRIGGER_TICKS LESS 7000 OR FOC_M0_TRIGGER_TICKS GREATER 8350)
        message(FATAL_ERROR "FOC_M0_TRIGGER_TICKS must be empty or in 7000..8350")
    endif()
endif()
if(NOT FOC_USB_DIVIDER MATCHES "^[1-9][0-9]*$" OR FOC_USB_DIVIDER GREATER 1000)
    message(FATAL_ERROR "FOC_USB_DIVIDER must be in 1..1000")
endif()
set_property(CACHE FOC_PROTECTION PROPERTY STRINGS WARN TRIP)
if(FOC_PROTECTION STREQUAL "WARN")
    set(FOC_PROTECTION_TRIP 0)
elseif(FOC_PROTECTION STREQUAL "TRIP")
    set(FOC_PROTECTION_TRIP 1)
else()
    message(FATAL_ERROR "FOC_PROTECTION must be WARN or TRIP")
endif()
set_property(CACHE FOC_PORT PROPERTY STRINGS M0 M1)
set_property(CACHE FOC_ENCODER PROPERTY STRINGS TLE5012B MT6835)
set_property(CACHE FOC_MOTOR PROPERTY STRINGS ZH3620_1 REFERENCE_24V)
set_property(CACHE FOC_LOAD_PROFILE PROPERTY STRINGS FREE_SHAFT PLASTIC_ARM)
if(NOT FOC_PORT MATCHES "^(M0|M1)$")
    message(FATAL_ERROR "FOC_PORT must be M0 or M1")
endif()
if(NOT FOC_ENCODER MATCHES "^(TLE5012B|MT6835)$")
    message(FATAL_ERROR "FOC_ENCODER must be TLE5012B or MT6835")
endif()
if(NOT FOC_MOTOR MATCHES "^(ZH3620_1|REFERENCE_24V)$")
    message(FATAL_ERROR "FOC_MOTOR must be ZH3620_1 or REFERENCE_24V")
endif()
if(NOT FOC_LOAD_PROFILE MATCHES "^(FREE_SHAFT|PLASTIC_ARM)$")
    message(FATAL_ERROR "FOC_LOAD_PROFILE must be FREE_SHAFT or PLASTIC_ARM")
endif()
if(FOC_MOTOR STREQUAL "REFERENCE_24V" AND FOC_LOAD_PROFILE STREQUAL "PLASTIC_ARM")
    message(FATAL_ERROR "PLASTIC_ARM gains are only characterized for ZH3620_1")
endif()
if(NOT FOC_INSTALLATION_ID MATCHES "^([1-9]|1[0-5])$")
    message(FATAL_ERROR "FOC_INSTALLATION_ID must be a decimal integer from 1 to 15")
endif()
if(FOC_DUAL)
    if(NOT FOC_PORT STREQUAL "M0" OR NOT FOC_ENCODER STREQUAL "TLE5012B")
        message(FATAL_ERROR "FOC_DUAL requires FOC_PORT=M0 and FOC_ENCODER=TLE5012B")
    endif()
    if(NOT FOC_M1_INSTALLATION_ID MATCHES "^([1-9]|1[0-5])$")
        message(FATAL_ERROR "FOC_M1_INSTALLATION_ID must be 1..15")
    endif()
endif()
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

add_executable(${CMAKE_PROJECT_NAME})
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    FOC_PORT_${FOC_PORT} FOC_ENCODER_${FOC_ENCODER}
    FOC_MOTOR_${FOC_MOTOR} FOC_LOAD_${FOC_LOAD_PROFILE} FOC_INSTALLATION_ID=${FOC_INSTALLATION_ID}
    FOC_PROTECTION_TRIP=${FOC_PROTECTION_TRIP} FOC_USB_DIVIDER=${FOC_USB_DIVIDER})
if(FOC_DUAL)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE FOC_DUAL FOC_M1_INSTALLATION_ID=${FOC_M1_INSTALLATION_ID})
endif()
if(FOC_PORT STREQUAL "M0")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE FOC_M0_DUAL_ADC=${FOC_M0_DUAL_ADC})
endif()
if(FOC_PORT STREQUAL "M0" AND NOT FOC_M0_TRIGGER_TICKS STREQUAL "")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE FOC_M0_TRIGGER_TICKS=${FOC_M0_TRIGGER_TICKS})
endif()
if(FOC_PORT STREQUAL "M0")
    set(FOC_ADC_SOURCE App/Hardware/bsp/bsp_adc_m0.c)
else()
    set(FOC_ADC_SOURCE App/Hardware/bsp/bsp_adc.c)
endif()
if(FOC_ENCODER STREQUAL "TLE5012B")
    set(FOC_ENCODER_SOURCE
        App/Hardware/encoder/bsp_encoder_tle5012b.c
        App/Hardware/tle5012b/tle5012b_protocol.c)
else()
    set(FOC_ENCODER_SOURCE
        App/Hardware/encoder/bsp_encoder_mt6835.c
        App/Hardware/mt6835/mt6835.c
        App/Hardware/mt6835/mt6835_port_stm32.c)
endif()
if(FOC_DUAL)
    set(FOC_ADC_DESCRIPTION "shared-two-rank")
else()
    set(FOC_ADC_DESCRIPTION "${FOC_M0_ADC_MODE}")
endif()
message(STATUS "FOC build: dual=${FOC_DUAL}, port=${FOC_PORT}, encoder=${FOC_ENCODER}, motor=${FOC_MOTOR}, load=${FOC_LOAD_PROFILE}, installation=${FOC_INSTALLATION_ID}, m1_installation=${FOC_M1_INSTALLATION_ID}, protection=${FOC_PROTECTION}, usb_divider=${FOC_USB_DIVIDER}, adc=${FOC_ADC_DESCRIPTION}")
if(DEFINED FOC_OUTPUT_NAME)
    set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
        OUTPUT_NAME "${FOC_OUTPUT_NAME}"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
endif()
add_subdirectory(cmake/stm32cubemx)
if(FOC_DUAL)
    set(FOC_CONTROL_SOURCES
        dual/app.c dual/hardware.c dual/record.c
        dual/foc0.c dual/foc1.c dual/control0.c dual/control1.c)
    set(FOC_SENSOR_SOURCES App/Hardware/tle5012b/tle5012b_protocol.c)
else()
    set(FOC_CONTROL_SOURCES
        App/Control/app.c App/Control/control.c App/FOC/foc.c
        App/Hardware/bsp/bsp_motor.c ${FOC_ADC_SOURCE})
    set(FOC_SENSOR_SOURCES ${FOC_ENCODER_SOURCE})
endif()
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${FOC_CONTROL_SOURCES}
    App/Hardware/bsp/bsp_can.c
    App/Hardware/bsp/bsp_uart.c
    App/Hardware/bsp/bsp_usb.c
    ${FOC_SENSOR_SOURCES}
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App/Control
    App/FOC
    App/Hardware/bsp
    App/Hardware/encoder
    App/Hardware/mt6835
    App/Hardware/tle5012b
    App/Config
    App/Protocols/JustFloat
    dual
)
option(FOC_CAPTURE "2048-sample current capture (separate debug build)" OFF)
if(FOC_CAPTURE)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE FOC_CAPTURE)
endif()
target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx m)
# USB FIFO service must stay short even in Debug builds.
target_compile_options(USB_Device_Library PRIVATE -O3)
set_source_files_properties(
    App/Hardware/bsp/bsp_usb.c USB_DEVICE/App/usbd_cdc_if.c
    USB_DEVICE/Target/usbd_conf.c
    PROPERTIES COMPILE_OPTIONS "-O3")
set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
    DIRECTORY cmake/stm32cubemx PROPERTIES COMPILE_OPTIONS "-O3")
# Keep the 20 kHz path optimized in Debug too; -O0 misses PWM deadlines.
set_source_files_properties(
    ${FOC_CONTROL_SOURCES} App/Hardware/bsp/bsp_uart.c
    ${FOC_SENSOR_SOURCES}
    PROPERTIES COMPILE_OPTIONS "-O3")
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/$<TARGET_FILE_BASE_NAME:${CMAKE_PROJECT_NAME}>.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/$<TARGET_FILE_BASE_NAME:${CMAKE_PROJECT_NAME}>.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    VERBATIM
)
