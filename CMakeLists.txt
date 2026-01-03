# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

set(EXTRA_COMPONENT_DIRS 
    "${CMAKE_CURRENT_SOURCE_DIR}/ascent-r3-drivers/bmp390l"
    "${CMAKE_CURRENT_SOURCE_DIR}/ascent-r3-drivers/comm/i2c"
    "${CMAKE_CURRENT_SOURCE_DIR}/ascent-r3-drivers/comm/spi"
    "${CMAKE_CURRENT_SOURCE_DIR}/ascent-r3-drivers/hardware_io/pyro"
    "${CMAKE_CURRENT_SOURCE_DIR}/ascent-r3-drivers/indicator"
    "${CMAKE_CURRENT_SOURCE_DIR}/ascent-r3-drivers/neopixel"
    "${CMAKE_CURRENT_SOURCE_DIR}/ascent-r3-drivers/patch/adc"
    "${CMAKE_CURRENT_SOURCE_DIR}/ascent-r3-drivers/psu"
    "${CMAKE_CURRENT_SOURCE_DIR}/ascent-r3-drivers/sam_m10q"
    "${CMAKE_CURRENT_SOURCE_DIR}/ascent-r3-drivers/w25qxx"
    "${CMAKE_CURRENT_SOURCE_DIR}/goober"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(ascent-r3-firmware)
