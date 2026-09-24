cmake_minimum_required(VERSION 3.22)
project(STM32F405-FOC C ASM)

# Both boards compile the same application. The primary project remains M0.
set(FOC_BOARD "M0" CACHE STRING "Motor/encoder board profile" FORCE)
set(FOC_OUTPUT_NAME "STM32F405-FOC")
add_subdirectory(variants/m1-mt6835)
