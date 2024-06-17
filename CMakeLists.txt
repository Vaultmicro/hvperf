cmake_minimum_required(VERSION 3.12)

project(
    hvperf
    VERSION 1.0.1
    LANGUAGES C
)

add_executable(hvperf hvperf_odroid.c usbstring.c)
add_compile_definitions(AIO)
target_link_libraries(hvperf pthread aio)