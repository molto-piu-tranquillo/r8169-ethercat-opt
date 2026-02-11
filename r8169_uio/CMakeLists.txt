cmake_minimum_required(VERSION 3.10)
project(r8169_uio C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -O2 -march=native")

include_directories(include)

# Library
add_library(r8169_uio STATIC
    src/r8169_uio.c
    src/r8169_dma.c
)

# EtherCAT test
add_executable(ethercat_test
    test/ethercat_test.c
)
target_link_libraries(ethercat_test r8169_uio)
