cmake_minimum_required(VERSION 3.22)
project(STM32F405-FOC C ASM)

include(cmake/foc-defaults.cmake)
set(FOC_OUTPUT_NAME "STM32F405-FOC")
add_subdirectory(variants/m1-mt6835)
