cmake_minimum_required(VERSION 3.22)
project(STM32F405-FOC C ASM)

# Default physical combination; override each axis with -D when configuring.
set(FOC_PORT "M0" CACHE STRING "Three-phase power connector: M0 or M1")
set(FOC_ENCODER "TLE5012B" CACHE STRING "Angle sensor model")
set(FOC_MOTOR "ZH3620_1" CACHE STRING "Motor model")
set(FOC_LOAD_PROFILE "PLASTIC_ARM" CACHE STRING "Mechanical load profile: FREE_SHAFT or PLASTIC_ARM")
set(FOC_INSTALLATION_ID "1" CACHE STRING "Physical sensor/magnet/phase installation: 1..15")
set(FOC_PROTECTION "WARN" CACHE STRING "Threshold policy: WARN or TRIP")
set(FOC_USB_DIVIDER "2" CACHE STRING "USB frame every N current samples")
set(FOC_OUTPUT_NAME "STM32F405-FOC")
option(FOC_DUAL "Run M0 and M1 together with TLE5012B encoders" ON)
add_subdirectory(variants/m1-mt6835)
