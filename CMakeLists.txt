cmake_minimum_required(VERSION 3.20.0)
# Add before find_package(Zephyr ...)
list(APPEND ZEPHYR_EXTRA_MODULES C:/ncs/v2.7.0/modules/arducam2)  # Replace with your exact path; use forward slashes for CMake compatibility
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(arducam_mega)
# -------------------------
# Suppress macro expansion notes
# -------------------------
add_compile_options(
    -fno-diagnostics-show-note-include-stack
    -fno-diagnostics-show-caret
    -fno-diagnostics-show-option
)
FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
target_include_directories(app PRIVATE include)
