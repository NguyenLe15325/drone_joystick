cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Make main/tusb_config.h visible to the tinyusb managed component,
# which does not list "main" in its own REQUIRES.
idf_build_set_property(COMPILE_OPTIONS "-I${CMAKE_CURRENT_LIST_DIR}/main" APPEND)

project(drone_joystick)
