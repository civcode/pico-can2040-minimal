# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)


# Include build functions from Pico SDK
set(PICO_SDK_PATH "/home/chris/pico/pico-sdk")
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C++ standards
project(test C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include_directories(
   ${CMAKE_SOURCE_DIR}/include
   ${CMAKE_SOURCE_DIR}/lib/can2040/src
   #${CMAKE_SOURCE_DIR}/../../../pico/pico-sdk/src/rp2_common/cmsis/stub/CMSIS/Device/RaspberryPi/RP2040/Include
)

set(SRC
   ${CMAKE_SOURCE_DIR}/src/main.c
   ${CMAKE_SOURCE_DIR}/lib/can2040/src/can2040.c
)

set(can2040_SRC
   ${CMAKE_SOURCE_DIR}/lib/can2040/src/can2040.c
)

add_library(can2040 
   ${can2040_SRC}
)

target_link_libraries(can2040 
   pico_base_headers
   pico_stdlib 
   hardware_pio 
   hardware_dma 
   cmsis_core
)
# target_sources(can2040 ${can2040_SRC})

#target_include_directories(can2040 ERFACE ${CMAKE_CURRENT_LIST_DIR}/src)
#target_link_libraries(can2040 INTERFACE pico_base_headers hardware_pio hardware_dma cmsis_core)
# message(STATUS "test")
# message(STATUS pico_base_headers)


#pico_generate_pio_header(can2040 ${CMAKE_CURRENT_LIST_DIR}/pio/can2040.pio)


# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/main.c ${SRC})

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# Link to pico_stdlib (gpio, time, etc. functions)
# target_link_libraries(${PROJECT_NAME} pico_base_headers pico_stdlib hardware_pio hardware_dma cmsis_core)
target_link_libraries(${PROJECT_NAME} pico_stdlib cmsis_core can2040)

# Enable usb output, disable uart output
#pico_enable_stdio_usb(${PROJECT_NAME} 1)
#pico_enable_stdio_uart(${PROJECT_NAME} 0)
